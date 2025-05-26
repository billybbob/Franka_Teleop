#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>

class ValueToSpeed : public rclcpp::Node {
public:
    // Récupération des valeurs cartésiennes avec quaternion
    struct CartesianPose {
        double x, y, z;           // Position
        double qx, qy, qz, qw;    // Quaternion
        double trigger;           // Valeur de la gâchette
    };

    ValueToSpeed() : Node("Value_to_Speed"), in_speed_mode_(false), trigger_value_(0.0), speed_factor_(1.0) {
        // Abonnement au topic valeur_effecteur (contient des valeurs cartésiennes avec quaternion)
        virtuose_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/valeur_effecteur", 10,
            std::bind(&ValueToSpeed::value_cartesian_callback, this, std::placeholders::_1)
        );

        // Abonnement au topic Mode_Position_Vitesse
        mode_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/Mode_Pose_Vitesse", 10,
            std::bind(&ValueToSpeed::mode_position_vitesse_callback, this, std::placeholders::_1)
        );
    
        // Création du publisher cmd_vel avec type Float32MultiArray pour les commandes cartésiennes (XYZ + Rx,Ry,Rz)
        cmd_vel_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "ValueToSpeed node initialized for cartesian control with quaternion support");
    }

private:
    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr virtuose_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mode_subscription;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_vel_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_publishers;
   
    // Constantes de configuration
    const double Vmax_base = 0.3;  // Vitesse maximale de base
    double speed_factor_;  // Facteur de vitesse provenant de la 4ème valeur du topic Mode_Pose_Vitesse
    double Vmax;  // Vitesse maximale ajustée (calculée comme Vmax_base * speed_factor_)

    // Valeur de la gâchette
    double trigger_value_;

    // Ajout d'un flag pour suivre l'état du mode
    bool in_speed_mode_;

    // Définition des Valeur à dépasser
    struct AxisLimits {
        double lower_limit;
        double upper_limit;
        double max_value;     // Valeur maximale utilisée pour la normalisation
        double min_value;     // Valeur minimale utilisée pour la normalisation
    };

    // Valeurs du contrôleur à dépasser pour mettre en mouvement le robot avec limites pour normalisation
    std::vector<AxisLimits> Value_for_deplacement = {
        {0.22, 0.28, 0.37, 0.13},               // Valeur à dépasser pour la translation suivant l'axe x (max 0.37, min  0.13)
        {-0.05, 0.01, 0.23, -0.27},              // Valeur à dépasser pour la translation suivant l'axe y (max 0.23, min -0.27)
        {0.01, 0.07, 0.19, -0.1},            // Valeur à dépasser pour la translation suivant l'axe z (max 0.19, min -0.1)
    };
    
    // Valeurs du contrôleur à dépasser pour les rotations en quaternion
    std::vector<AxisLimits> Value_for_rotation = {
        {-0.2, 0.2, 0.8, -0.8},               // Rotation autour de l'axe x (Roll)
        {0.4, 0.8, 1.5, -0.3},               // Rotation autour de l'axe y (Pitch)
        {-0.225, 0.225, 0.9, -0.9},               // Rotation autour de l'axe z (Yaw)
    };

    // Ajout du callback pour le topic Mode_Position_Vitesse
    void mode_position_vitesse_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Message du mode invalide: attendu au moins 2 valeurs, reçu %zu", msg->data.size());
            return;
        }
        
        // Dans le message: [0] = mode position (1.0 = actif), [1] = mode vitesse (1.0 = actif)
        bool position_mode = msg->data[0] > 0.5;
        bool speed_mode = msg->data[1] > 0.5;
        
        // Mettre à jour le mode uniquement s'il a changé
        if (speed_mode != in_speed_mode_) {
            in_speed_mode_ = speed_mode;
            RCLCPP_INFO(this->get_logger(), "Mode changé: %s", in_speed_mode_ ? "Mode Vitesse" : "Mode Position");
        }
        
        // Récupérer la 4ème valeur si elle existe (à l'index 3)
        if (msg->data.size() >= 4) {
            speed_factor_ = static_cast<double>(msg->data[3]);
            Vmax = Vmax_base * speed_factor_;
            RCLCPP_INFO(this->get_logger(), "Facteur de vitesse mis à jour: %f, Vmax ajusté: %f", speed_factor_, Vmax);
        } else {
            // Si la 4ème valeur n'existe pas, utiliser la valeur par défaut
            speed_factor_ = 1.0;
            Vmax = Vmax_base;
        }
    }

    // Fonction utilitaire pour normaliser la vitesse
    double normalizeSpeed(double value, double threshold, double max_value, bool positive) {
        if (positive) {
            // Pour les valeurs au-dessus du seuil supérieur
            return Vmax * ((value - threshold) / (max_value - threshold));
        } else {
            // Pour les valeurs en-dessous du seuil inférieur
            return -Vmax * ((threshold - value) / (threshold - max_value));
        }
    }

    // Fonction pour limiter la vitesse à la plage [-Vmax, Vmax]
    double clampSpeed(double speed) {
        if (speed > Vmax) return Vmax;
        if (speed < -Vmax) return -Vmax;
        return speed;
    }

    // Fonction pour convertir un quaternion en angles d'Euler (roll, pitch, yaw)
    std::vector<double> quaternionToEuler(double qx, double qy, double qz, double qw) {
        std::vector<double> euler(3, 0.0);
        
        // Roll (rotation autour de X)
        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        euler[0] = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (rotation autour de Y)
        double sinp = 2.0 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1)
            euler[1] = std::copysign(M_PI / 2, sinp); // utiliser 90 degrés si hors limites
        else
            euler[1] = std::asin(sinp);
        
        // Yaw (rotation autour de Z)
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        euler[2] = std::atan2(siny_cosp, cosy_cosp);
        
        return euler;
    }

    // Callback pour le traitement des valeurs cartésiennes avec quaternion
    void value_cartesian_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // Ne traiter les valeurs que si nous sommes en mode vitesse
        if (!in_speed_mode_) {
            // Nous sommes en mode position, ne rien faire
            return;
        }

        if (msg->data.size() < 8) {
            RCLCPP_ERROR(this->get_logger(), "Message reçu ne contient pas assez de données (attendu 8, reçu %zu)", msg->data.size());
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Reçu valeurs cartésienne avec quaternion: [%f, %f, %f, %f, %f, %f, %f]", 
            msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
        
        // Extraire les valeurs cartésiennes du message
        std::vector<double> cartesian_values;
        for (size_t i = 0; i < msg->data.size() && i < 8; ++i) {
            cartesian_values.push_back(static_cast<double>(msg->data[i]));
        }

        // Structure pour stocker les valeurs cartésiennes plus clairement
        CartesianPose pose = {
            cartesian_values[0], cartesian_values[1], cartesian_values[2],  // Position (x, y, z)
            cartesian_values[3], cartesian_values[4], cartesian_values[5], cartesian_values[6],  // Quaternion (qx, qy, qz, qw)
            cartesian_values[7]  // Valeur de la gâchette
        };

        // Mettre à jour la valeur de la gâchette (valeur entre 0 et 1 typiquement)
        trigger_value_ = pose.trigger;

        // Calculer les vitesses pour la position (x, y, z)
        std::vector<double> linear_speeds(3, 0.0);
        
        // Traiter les 3 translations
        for (size_t i = 0; i < 3; ++i) {
            double value = cartesian_values[i];
            const auto& limits = Value_for_deplacement[i];
            
            if (value > limits.upper_limit) {
                linear_speeds[i] = normalizeSpeed(value, limits.upper_limit, limits.max_value, true);
            } else if (value < limits.lower_limit) {
                linear_speeds[i] = normalizeSpeed(value, limits.lower_limit, limits.min_value, false);
            }
            
            linear_speeds[i] = clampSpeed(linear_speeds[i]);
        }
        
        // Appliquer la rotation de pi autour de l'axe X pour passer du repère contrôleur au repère robot
        // X reste le même, Y devient -Y, Z devient -Z
        linear_speeds[1] = -linear_speeds[1];  // Y est inversé
        linear_speeds[2] = -linear_speeds[2];  // Z est inversé
        
        // Calculer les angles d'Euler à partir du quaternion
        std::vector<double> euler_angles = quaternionToEuler(
            pose.qx, pose.qy, pose.qz, pose.qw);
            
        // Calculer les vitesses angulaires
        std::vector<double> angular_speeds(3, 0.0);
        
        // Traiter les 3 rotations
        for (size_t i = 0; i < 3; ++i) {
            double value = euler_angles[i];
            const auto& limits = Value_for_rotation[i];
            
            if (value > limits.upper_limit) {
                angular_speeds[i] = normalizeSpeed(value, limits.upper_limit, limits.max_value, true);
            } else if (value < limits.lower_limit) {
                angular_speeds[i] = normalizeSpeed(value, limits.lower_limit, limits.min_value, false);
            }
            
            angular_speeds[i] = clampSpeed(angular_speeds[i]);
        }
        
        // Appliquer la rotation de pi autour de l'axe X pour les rotations également
        // Roll reste le même, Pitch devient -Pitch, Yaw devient -Yaw
        angular_speeds[1] = -angular_speeds[1];  // Pitch est inversé
        angular_speeds[2] = -angular_speeds[2];  // Yaw est inversé

        // Calcul vitesse de la gâchette (en utilisant Vmax ajusté)
        trigger_value_ = pose.trigger * Vmax;

        // Créer et publier le message Float32MultiArray pour cmd_vel
        auto cmd_vel_msg = std_msgs::msg::Float32MultiArray();
        
        // Configuration du layout du message
        cmd_vel_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        cmd_vel_msg.layout.dim[0].label = "velocities";
        cmd_vel_msg.layout.dim[0].size = 7;  // 3 linéaires + 3 angulaires + gâchette
        cmd_vel_msg.layout.dim[0].stride = 7;
        cmd_vel_msg.layout.data_offset = 0;
        
        // Ajouter les vitesses au message
        cmd_vel_msg.data.resize(7);
        cmd_vel_msg.data[0] = static_cast<float>(linear_speeds[0]);  // X
        cmd_vel_msg.data[1] = static_cast<float>(linear_speeds[1]);  // Y
        cmd_vel_msg.data[2] = static_cast<float>(linear_speeds[2]);  // Z
        cmd_vel_msg.data[3] = static_cast<float>(angular_speeds[0]); // Roll
        cmd_vel_msg.data[4] = static_cast<float>(angular_speeds[1]); // Pitch
        cmd_vel_msg.data[5] = static_cast<float>(angular_speeds[2]); // Yaw
        cmd_vel_msg.data[6] = static_cast<float>(trigger_value_);    // Valeur de la gâchette

        // Publier sur cmd_vel
        cmd_vel_->publish(cmd_vel_msg);

        // Afficher les vitesses calculées (pour le débogage)
        RCLCPP_INFO(this->get_logger(), "Vitesses linéaires publiées XYZ: [%f, %f, %f] avec Vmax: %f",
            cmd_vel_msg.data[0], cmd_vel_msg.data[1], cmd_vel_msg.data[2], Vmax);
        RCLCPP_INFO(this->get_logger(), "Vitesses angulaires publiées RPY: [%f, %f, %f]",
            cmd_vel_msg.data[3], cmd_vel_msg.data[4], cmd_vel_msg.data[5]);
        RCLCPP_INFO(this->get_logger(), "Vitesses de la gâchette: %f",
            cmd_vel_msg.data[6]);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ValueToSpeed>());
    rclcpp::shutdown();
    return 0;
}
