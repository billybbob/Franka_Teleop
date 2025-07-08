#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <vector>

class Offset_Position : public rclcpp::Node {
public:
    // Récupération des valeurs cartésiennes avec quaternion
    struct CartesianPose {
        double x, y, z;           // Position
        double qx, qy, qz, qw;    // Orientation (quaternion)
    };

    Offset_Position() : Node("Offset_Position"), in_pose_mode_(false), initial_reference_set_(false), coeff_homo_(1.0) {
        // Abonnement au topic valeur_effecteur (contient des valeurs cartésiennes avec quaternion)
        virtuose_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/valeur_effecteur", 10,
            std::bind(&Offset_Position::value_cartesian_callback, this, std::placeholders::_1)
        );

        // Abonnement au topic position_effecteur_robot (contient des valeurs cartésiennes avec quaternion)
        robot_subscription = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/position_effecteur_robot", 10,
            std::bind(&Offset_Position::position_effecteur_robot_callback, this, std::placeholders::_1)
        );

        // Abonnement au topic Mode_Position_Vitesse
        mode_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/Mode_Pose_Vitesse", 10,
            std::bind(&Offset_Position::mode_position_vitesse_callback, this, std::placeholders::_1)
        );
    
        // Création du publisher cmd_pose pour les commandes cartésiennes avec quaternions
        cmd_pose_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/cmd_pose", 10);

        // Initialiser la position du robot
        robot_pose_.x = 0.0;
        robot_pose_.y = 0.0;
        robot_pose_.z = 0.0;
        robot_pose_.qx = 0.0;
        robot_pose_.qy = 0.0;
        robot_pose_.qz = 0.0;
        robot_pose_.qw = 0.0;

        // Initialiser la position du contrôleur
        controller_pose_.x = 0;
        controller_pose_.y = 0;
        controller_pose_.z = 0;
        controller_pose_.qx = 0;
        controller_pose_.qy = 0;
        controller_pose_.qz = 0;
        controller_pose_.qw = 1;

        // Initialiser la position de référence (sera mise à jour lors du passage en mode position)
        reference_robot_pose_ = robot_pose_;
        reference_controller_pose_ = controller_pose_;

        // Variable afin d'enregistrer la position du robot seulement au moment où l'on passe en mode position
        pose_enregistrer_ = 0.0;

        // Drapeau pour indiquer si nous avons reçu les deux positions
        robot_pose_received_ = false;
        controller_pose_received_ = false;

        // Créer un timer pour vérifier périodiquement si nous sommes prêts à envoyer des commandes
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&Offset_Position::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Offset_Position node initialized for cartesian control");
    }

private:
    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr virtuose_subscription;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr robot_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr mode_subscription;

    // Initialize count
    int count_ = 0;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_pose_;

    // Timer pour vérifier périodiquement l'état
    rclcpp::TimerBase::SharedPtr timer_;

    // Variables pour stocker les positions initiale
    CartesianPose robot_pose_;        // Position initiale de l'effecteur du robot
    CartesianPose controller_pose_;   // Position initiale du contrôleur (virtuose)

    // Variables pour stocker les positions de référence (enregistrées au passage en mode position)
    CartesianPose reference_robot_pose_;            // Position de référence de l'effecteur du robot
    CartesianPose reference_controller_pose_;       // Position de référence du contrôleur
    CartesianPose last_robot_pose_in_speed_mode_;   // Dernière position de l'effecteur du robot


    CartesianPose last_sent_pose_;

    // Variable afin d'enregistrer la position du robot seulement au moment où l'on passe en mode position
    double pose_enregistrer_;

    // Variable pour définir le coefficient d'homothétie
    double coeff_homo_;

    // Drapeaux pour suivre les réceptions de données
    bool robot_pose_received_;
    bool controller_pose_received_;

    // Ajout d'un flag pour suivre l'état du mode
    bool in_pose_mode_;
    
    // Flag pour indiquer si la référence initiale a été définie
    bool initial_reference_set_;

    // Callback du timer pour vérifier périodiquement l'état
    void timer_callback() {
        // Si nous sommes en mode position, les références sont définies et nous avons reçu les positions courantes
        if (in_pose_mode_ && initial_reference_set_ && robot_pose_received_ && controller_pose_received_) {
            // Calculer et envoyer la commande
            calculateAndSendCommand();
        }
    }

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
        if (position_mode != in_pose_mode_) {
            in_pose_mode_ = position_mode;
            RCLCPP_INFO(this->get_logger(), "Mode changé: %s", in_pose_mode_ ? "Mode Position" : "Mode Vitesse");
            
            // Réinitialiser le flag de référence lors du passage en mode vitesse
            if (!in_pose_mode_) {
                initial_reference_set_ = false;
                RCLCPP_INFO(this->get_logger(), "Mode vitesse activé, référence initiale réinitialisée");
            }else {
                // Nous passons en mode position : réutiliser la dernière position enregistrée
                reference_robot_pose_ = last_robot_pose_in_speed_mode_;
                RCLCPP_INFO(this->get_logger(), "Nouvelle référence du robot définie depuis le mode vitesse: [%f, %f, %f] [%f, %f, %f, %f]",
                    reference_robot_pose_.x, reference_robot_pose_.y, reference_robot_pose_.z,
                    reference_robot_pose_.qx, reference_robot_pose_.qy, reference_robot_pose_.qz, reference_robot_pose_.qw);
            }
        }

        // Récupérer le coefficient d'homothétie s'il est présent (4ème valeur à l'index 3)
        if (msg->data.size() >= 4) {
            double new_coeff_homo = static_cast<double>(msg->data[3]);
            if (new_coeff_homo != coeff_homo_) {
                coeff_homo_ = new_coeff_homo;
                RCLCPP_INFO(this->get_logger(), "Coefficient d'homothétie mis à jour: %f", coeff_homo_);
            }
        }

        if (position_mode == 1){
            pose_enregistrer_ = 1;
        } else {
            pose_enregistrer_ = 0;
        }
    }

    // Callback pour recevoir la position de l'effecteur du robot
    void position_effecteur_robot_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        if (msg->transforms.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Message de position robot reçu est vide (aucune transformation)");
            return;
        }

        // Récupérer la première transformation du message
        const auto& transform = msg->transforms[0];

        // Mettre à jour la position du robot
        robot_pose_.x = transform.transform.translation.x;
        robot_pose_.y = transform.transform.translation.y;
        robot_pose_.z = transform.transform.translation.z;
        robot_pose_.qx = transform.transform.rotation.x;
        robot_pose_.qy = transform.transform.rotation.y;
        robot_pose_.qz = transform.transform.rotation.z;
        robot_pose_.qw = transform.transform.rotation.w;

        robot_pose_received_ = true;

        // Enregistrer la position courante du robot si on est en mode vitesse
        if (!in_pose_mode_) {
            last_robot_pose_in_speed_mode_ = robot_pose_;
        }

        // Si nous venons de passer en mode position et n'avons pas encore défini la référence
        if (in_pose_mode_ && pose_enregistrer_ == 1 && !initial_reference_set_) {
            // Enregistrer la position de référence du robot
            reference_robot_pose_ = robot_pose_;
            
            RCLCPP_INFO(this->get_logger(), "Position de référence du robot enregistrée: [%f, %f, %f] [%f, %f, %f, %f]",
                reference_robot_pose_.x, reference_robot_pose_.y, reference_robot_pose_.z,
                reference_robot_pose_.qx, reference_robot_pose_.qy, reference_robot_pose_.qz, reference_robot_pose_.qw);
            
            // Vérifier si nous avons également enregistré la position de référence du contrôleur
            if (controller_pose_received_) {
                initial_reference_set_ = true;
                RCLCPP_INFO(this->get_logger(), "Références initiales définies, prêt à envoyer des commandes");
            }
        }
    }

    // Callback pour recevoir la position de l'effecteur du contrôleur
    void value_cartesian_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        // Vérifier si le message contient suffisamment de données
        if (msg->data.size() < 7) {
            RCLCPP_ERROR(this->get_logger(), "Message du contrôleur reçu ne contient pas assez de données (attendu 7, reçu %zu)", msg->data.size());
            return;
        }

        // Mettre à jour la position du contrôleur et normalisation des valeurs, afin d'avoir 0 comme valeur milieu
        controller_pose_.x = static_cast<double>(msg->data[0]) - 0.25;
        controller_pose_.y = static_cast<double>(msg->data[1]) + 0.02;
        controller_pose_.z = static_cast<double>(msg->data[2]) - 0.05;
        controller_pose_.qx = static_cast<double>(msg->data[3]) - 0.0;
        controller_pose_.qy = static_cast<double>(msg->data[4]) - 0.472458;
        controller_pose_.qz = static_cast<double>(msg->data[5]) + 0.111235;
        controller_pose_.qw = static_cast<double>(msg->data[6]);

        controller_pose_received_ = true;

        // Si nous venons juste de passer en mode position et n'avons pas encore défini la référence
        if (in_pose_mode_ && pose_enregistrer_ == 1 && !initial_reference_set_) {
            // Enregistrer la position de référence du contrôleur
            reference_controller_pose_ = controller_pose_;
            
            RCLCPP_INFO(this->get_logger(), "Position de référence du contrôleur enregistrée: [%f, %f, %f] [%f, %f, %f, %f]",
                reference_controller_pose_.x, reference_controller_pose_.y, reference_controller_pose_.z,
                reference_controller_pose_.qx, reference_controller_pose_.qy, reference_controller_pose_.qz, reference_controller_pose_.qw);
            
            // Vérifier si nous avons également enregistré la position de référence du robot
            if (robot_pose_received_) {
                initial_reference_set_ = true;
                RCLCPP_INFO(this->get_logger(), "Références initiales définies, prêt à envoyer des commandes");
            }
        }
    }
    
    // Fonction pour calculer et envoyer la commande de position
    void calculateAndSendCommand() {
        // Créer et publier le message Float32MultiArray pour cmd_pose
        auto twist_msg = std_msgs::msg::Float32MultiArray();

        // Effectuer la rotation du delta de contrôleur pour l'aligner avec le repère du robot
        CartesianPose rotated_delta = controller_pose_;
        
        // On remplit le vecteur 'data' avec les valeurs transformées
        twist_msg.data.resize(7);  // 7 valeurs: x, y, z, qx, qy, qz, qw
        
        // Configuration du layout du message (optionnel mais recommandé)
        twist_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        twist_msg.layout.dim[0].label = "pose";
        twist_msg.layout.dim[0].size = 7;  // 3 positions + 4 quaternion
        twist_msg.layout.dim[0].stride = 7;
        twist_msg.layout.data_offset = 0;
        
        // Appliquer les transformations des positions à partir de la position de référence du robot
        twist_msg.data[0] = reference_robot_pose_.x + (rotated_delta.x * coeff_homo_);
        twist_msg.data[1] = reference_robot_pose_.y + (rotated_delta.y * coeff_homo_);
        twist_msg.data[2] = reference_robot_pose_.z + (rotated_delta.z * coeff_homo_);

        // Calculer l'orientation finale en utilisant le quaternion transformé
        twist_msg.data[3] = reference_robot_pose_.qx * (rotated_delta.qx * coeff_homo_);
        twist_msg.data[4] = reference_robot_pose_.qy * (rotated_delta.qy * coeff_homo_);
        twist_msg.data[5] = reference_robot_pose_.qz * (rotated_delta.qz * coeff_homo_);
        twist_msg.data[6] = rotated_delta.qw;


        // Mettre à jour la dernière position envoyée
        last_sent_pose_ = controller_pose_;

        // Publier sur cmd_pose
        cmd_pose_->publish(twist_msg);


        if (count_ % 1000 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Commande de position envoyée : [%f, %f, %f] avec coeff_homo_: %f",
                twist_msg.data[0], twist_msg.data[1], twist_msg.data[2], coeff_homo_);
            RCLCPP_INFO(this->get_logger(), "Quaternions envoyés: [%f, %f, %f, %f]",
                twist_msg.data[3], twist_msg.data[4], twist_msg.data[5], twist_msg.data[6]);
        }

        count_++;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Offset_Position>());
    rclcpp::shutdown();
    return 0;
}