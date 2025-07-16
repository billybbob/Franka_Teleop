/**
 * @file gripper_example_controller_simu.cpp
 * @brief Contrôleur de pince pour robot Franka FR3 avec support de téléopération haptique
 * 
 * Ce contrôleur implémente un système de contrôle bidirectionnel pour la pince du robot Franka FR3.
 * Il supporte deux modes de contrôle :
 * - Contrôle en vitesse : mouvement continu avec inversion automatique aux limites
 * - Contrôle en position : positionnement précis avec contrôle proportionnel
 * 
 * Le contrôleur est conçu pour être utilisé avec un dispositif haptique Desktop 6D de Haption
 * dans le cadre d'un projet de téléopération robotique sous ROS2 Humble et Gazebo Fortress.
 * 
 * @author Vincent Bassemayousse
 * @date 07/10/2025
 * @version 1.0
 */

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "fmt/format.h"
#include "franka_example_controllers/gripper_example_controller_simu.hpp"

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Définition des couleurs pour l'affichage console
#define GREEN "\033[1;32m"
#define RESET "\033[0m"

namespace franka_example_controllers {

/**
 * @brief Configure les interfaces de commande pour le contrôleur de pince
 * 
 * Cette méthode définit les interfaces nécessaires pour commander les deux doigts
 * de la pince Franka FR3. Les interfaces sont configurées en mode INDIVIDUAL
 * pour permettre un contrôle indépendant de chaque doigt.
 * 
 * @return InterfaceConfiguration Configuration des interfaces de commande
 * @note Les interfaces configurées sont :
 *       - fr3_finger_joint1/velocity : Contrôle en vitesse du doigt 1
 *       - fr3_finger_joint2/velocity : Contrôle en vitesse du doigt 2
 */
controller_interface::InterfaceConfiguration
GripperExampleControllerSimu::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Configuration des interfaces de commande pour les deux doigts de la pince
  config.names.push_back("fr3_finger_joint1/velocity");  // Doigt 1 - Commande en vitesse
  config.names.push_back("fr3_finger_joint2/velocity");  // Doigt 2 - Commande en vitesse
  
  return config;
}

/**
 * @brief Configure les interfaces d'état pour le contrôleur de pince
 * 
 * Cette méthode définit les interfaces nécessaires pour lire l'état des deux doigts
 * de la pince. Ces informations sont utilisées pour la rétroaction et le contrôle.
 * 
 * @return InterfaceConfiguration Configuration des interfaces d'état
 * @note Les interfaces configurées sont :
 *       - fr3_finger_joint1/velocity : Lecture de vitesse du doigt 1
 *       - fr3_finger_joint2/velocity : Lecture de vitesse du doigt 2
 */
controller_interface::InterfaceConfiguration
GripperExampleControllerSimu::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Configuration des interfaces d'état pour les deux doigts de la pince
  config.names.push_back("fr3_finger_joint1/velocity");  // Doigt 1 - Lecture de vitesse
  config.names.push_back("fr3_finger_joint2/velocity");  // Doigt 2 - Lecture de vitesse
  
  return config;
}

/**
 * @brief Initialise le contrôleur et déclare les paramètres configurables
 * 
 * Cette méthode est appelée lors de l'initialisation du contrôleur. Elle déclare
 * tous les paramètres configurables avec leurs valeurs par défaut.
 * 
 * @return CallbackReturn::SUCCESS si l'initialisation réussit, ERROR sinon
 * @throws std::exception Si une erreur survient lors de la déclaration des paramètres
 * 
 * @note Paramètres déclarés :
 *       - default_velocity : Vitesse par défaut (0.0 m/s)
 *       - max_opening : Ouverture maximale de la pince (0.04 m)
 *       - min_opening : Ouverture minimale de la pince (0.0 m)
 *       - position_threshold : Seuil de détection de position (0.002 m)
 *       - position_gain : Gain du contrôleur proportionnel (5.0)
 */
CallbackReturn GripperExampleControllerSimu::on_init() {
  try {
    // Déclaration des paramètres configurables du contrôleur
    auto_declare<double>("default_velocity", 0.0);       // Vitesse par défaut si aucune commande
    auto_declare<double>("max_opening", 0.04);           // Ouverture maximale autorisée (4 cm)
    auto_declare<double>("min_opening", 0.0);            // Ouverture minimale (pince fermée)
    auto_declare<double>("position_threshold", 0.002);   // Seuil pour détecter les limites (2 mm)
    auto_declare<double>("position_gain", 5.0);          // Gain proportionnel pour contrôle position
  } catch (const std::exception& e) {
    // Gestion des erreurs d'initialisation
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

/**
 * @brief Configure le contrôleur avec les paramètres et initialise les abonnements
 * 
 * Cette méthode configure le contrôleur en récupérant les paramètres et en initialisant
 * les variables internes. Elle crée également les abonnements ROS2 pour recevoir les
 * commandes de téléopération.
 * 
 * @param state État du cycle de vie du contrôleur (non utilisé)
 * @return CallbackReturn::SUCCESS si la configuration réussit
 * 
 * @note Abonnements créés :
 *       - /gripper_command_velocities : Commandes de vitesse depuis le dispositif haptique
 *       - /gripper_command_positions : Commandes de position depuis le dispositif haptique
 *       - /joint_states : État des joints pour la rétroaction
 */
CallbackReturn GripperExampleControllerSimu::on_configure(const rclcpp_lifecycle::State&) {
  // Récupération des paramètres configurables
  default_velocity_ = get_node()->get_parameter("default_velocity").as_double();
  current_velocity_ = default_velocity_;
  max_opening_ = get_node()->get_parameter("max_opening").as_double();
  min_opening_ = get_node()->get_parameter("min_opening").as_double();
  position_threshold_ = get_node()->get_parameter("position_threshold").as_double();
  position_gain_ = get_node()->get_parameter("position_gain").as_double();
  
  // Initialisation des positions des doigts (état initial)
  finger1_position_ = 0.0;  // Position du doigt 1
  finger2_position_ = 0.0;  // Position du doigt 2
  
  // Initialisation de la position cible pour le contrôle en position
  target_position_ = 0.0;
  
  // Mode de contrôle : 0 = vitesse, 1 = position
  control_mode_ = 0;
  
  // Direction du mouvement : 1 = ouverture, -1 = fermeture
  direction_ = 1;

  // Création de l'abonnement aux commandes de vitesse depuis le dispositif haptique
  velocity_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      "/gripper_command_velocities", 10,
      std::bind(&GripperExampleControllerSimu::velocityCommandCallback, this, std::placeholders::_1));

  // Création de l'abonnement aux commandes de position depuis le dispositif haptique
  position_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "/gripper_command_positions", 10,
    std::bind(&GripperExampleControllerSimu::positionCommandCallback, this, std::placeholders::_1));

  // Création de l'abonnement aux états des joints pour la rétroaction
  joint_states_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&GripperExampleControllerSimu::jointstatesCallback, this, std::placeholders::_1));
  
  // Messages informatifs de configuration
  RCLCPP_INFO(get_node()->get_logger(), 
              "Gripper controller with position and velocity control configured.");
  RCLCPP_INFO(get_node()->get_logger(), 
              "Subscribed to /gripper_command_velocities, /gripper_command_positions and /joint_states");
  RCLCPP_INFO(get_node()->get_logger(), 
              "Using default velocity: %f, max_opening: %f, position_gain: %f", 
              default_velocity_, max_opening_, position_gain_);
  
  return CallbackReturn::SUCCESS;
}

/**
 * @brief Callback pour les commandes de vitesse provenant du dispositif haptique
 * 
 * Cette méthode traite les commandes de vitesse reçues depuis le dispositif haptique
 * Desktop 6D. Elle commute le contrôleur en mode vitesse et met à jour la magnitude
 * de vitesse tout en préservant la direction automatique.
 * 
 * @param msg Message contenant la commande de vitesse
 * @note Le contrôleur utilise une logique de direction automatique :
 *       - La magnitude de vitesse est définie par la commande haptique
 *       - La direction est gérée automatiquement selon les limites de la pince
 *       - En cas de valeur invalide, utilise la vitesse par défaut
 */
void GripperExampleControllerSimu::velocityCommandCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  // Commutation vers le mode de contrôle en vitesse
  control_mode_ = 0;
  
  // Traitement de la commande de vitesse reçue
  if (msg->data > 0.0) {
    // Mise à jour de la magnitude de vitesse (la direction est gérée automatiquement)
    current_velocity_magnitude_ = msg->data;
    RCLCPP_DEBUG(get_node()->get_logger(), 
                "Switched to velocity mode. Updated gripper velocity magnitude to: %f", 
                current_velocity_magnitude_);
    
    // Application de la direction à la magnitude pour obtenir la vitesse finale
    current_velocity_ = direction_ * current_velocity_magnitude_;
  } else {
    // Gestion des valeurs invalides - utilisation de la vitesse par défaut
    RCLCPP_DEBUG(get_node()->get_logger(), 
                "Received invalid velocity value: %f. Using default: %f", 
                msg->data, default_velocity_);
    current_velocity_magnitude_ = std::abs(default_velocity_);
    current_velocity_ = direction_ * current_velocity_magnitude_;
  }
}

/**
 * @brief Callback pour les commandes de position provenant du dispositif haptique
 * 
 * Cette méthode traite les commandes de position reçues depuis le dispositif haptique.
 * Elle commute le contrôleur en mode position et valide la position cible demandée
 * en la clamant dans les limites autorisées.
 * 
 * @param msg Message contenant la commande de position cible
 * @note La position est automatiquement clampée entre min_opening et max_opening
 *       pour garantir la sécurité de la pince et éviter les dommages mécaniques
 */
void GripperExampleControllerSimu::positionCommandCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  // Commutation vers le mode de contrôle en position
  control_mode_ = 1;
  
  double requested_position = msg->data;
  
  // Validation et clamping de la position demandée dans les limites autorisées
  if (requested_position > max_opening_) {
    target_position_ = max_opening_;
    RCLCPP_DEBUG(get_node()->get_logger(), 
                "Requested position %f exceeds max opening. Clamping to %f", 
                requested_position, max_opening_);
  } else if (requested_position < min_opening_) {
    target_position_ = min_opening_;
    RCLCPP_DEBUG(get_node()->get_logger(), 
                "Requested position %f below min opening. Clamping to %f", 
                requested_position, min_opening_);
  } else {
    target_position_ = requested_position;
  }
  
  RCLCPP_DEBUG(get_node()->get_logger(), 
              "Switched to position mode. Target position set to: %f", target_position_);
}

/**
 * @brief Callback pour la rétroaction des états des joints
 * 
 * Cette méthode traite les messages d'état des joints pour obtenir la position
 * actuelle des doigts de la pince. Elle implémente la logique d'inversion automatique
 * de direction en mode vitesse lorsque les limites sont atteintes.
 * 
 * @param msg Message contenant l'état de tous les joints du robot
 * @note Fonctionnalités implementées :
 *       - Extraction des positions des doigts depuis le message joint_state
 *       - Calcul de la position moyenne pour une détection robuste
 *       - Inversion automatique de direction aux limites en mode vitesse
 *       - Rétroaction pour le contrôle en position
 */
void GripperExampleControllerSimu::jointstatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  // Recherche des indices des joints des doigts dans le message
  int finger1_index = -1;
  int finger2_index = -1;
  
  // Parcours des noms de joints pour trouver ceux de la pince
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "fr3_finger_joint1") {
      finger1_index = i;
    } else if (msg->name[i] == "fr3_finger_joint2") {
      finger2_index = i;
    }
    
    // Optimisation : arrêt de la recherche si les deux indices sont trouvés
    if (finger1_index != -1 && finger2_index != -1) {
      break;
    }
  }
  
  // Traitement des positions si les joints sont trouvés
  if (finger1_index != -1 && finger2_index != -1) {
    // Mise à jour des positions individuelles des doigts
    finger1_position_ = msg->position[finger1_index];
    finger2_position_ = msg->position[finger2_index];
    
    // Calcul de la position moyenne pour une détection plus robuste
    double avg_position = (finger1_position_ + finger2_position_) / 2.0;
    
    // Logique d'inversion automatique en mode vitesse
    if (control_mode_ == 0) {  // Mode contrôle en vitesse
      // Vérification si la pince atteint l'ouverture maximale
      if (direction_ > 0 && avg_position >= (max_opening_ - position_threshold_)) {
        // Inversion vers la fermeture
        direction_ = -1;
        current_velocity_ = direction_ * current_velocity_magnitude_;
        RCLCPP_DEBUG(get_node()->get_logger(), 
                   "Max opening reached (%f). Reversing to closing direction.", avg_position);
      } 
      // Vérification si la pince atteint l'ouverture minimale
      else if (direction_ < 0 && avg_position <= (min_opening_ + position_threshold_)) {
        // Inversion vers l'ouverture
        direction_ = 1;
        current_velocity_ = direction_ * current_velocity_magnitude_;
        RCLCPP_DEBUG(get_node()->get_logger(), 
                   "Min opening reached (%f). Reversing to opening direction.", avg_position);
      }
    }

    // Logging des informations de debug
    RCLCPP_DEBUG(get_node()->get_logger(), 
                "Gripper positions: f1=%f, f2=%f, avg=%f, mode=%d", 
                finger1_position_, finger2_position_, avg_position, control_mode_);
  }
}

/**
 * @brief Active le contrôleur et initialise les interfaces de commande
 * 
 * Cette méthode est appelée lors de l'activation du contrôleur. Elle initialise
 * les interfaces de commande avec la vitesse par défaut et vérifie que toutes
 * les interfaces nécessaires sont disponibles.
 * 
 * @param state État du cycle de vie du contrôleur (non utilisé)
 * @return CallbackReturn::SUCCESS si l'activation réussit, ERROR sinon
 * @note Vérification de sécurité : s'assure que les 2 interfaces de commande
 *       sont disponibles avant l'activation
 */
CallbackReturn GripperExampleControllerSimu::on_activate(const rclcpp_lifecycle::State&) {
  // Vérification de la disponibilité des interfaces de commande
  if (command_interfaces_.size() >= 2) {
    // Initialisation des deux doigts avec la vitesse par défaut
    command_interfaces_[0].set_value(current_velocity_); // Doigt 1
    command_interfaces_[1].set_value(current_velocity_); // Doigt 2
    RCLCPP_INFO(get_node()->get_logger(), "Both gripper command interfaces activated");
  } else {
    // Erreur critique : interfaces insuffisantes
    RCLCPP_ERROR(get_node()->get_logger(), 
                "Not enough command interfaces found! Expected 2, got %zu", 
                command_interfaces_.size());
    return CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}

/**
 * @brief Désactive le contrôleur et arrête les mouvements
 * 
 * Cette méthode est appelée lors de la désactivation du contrôleur. Elle arrête
 * tous les mouvements en mettant la vitesse à zéro pour garantir la sécurité.
 * 
 * @param state État du cycle de vie du contrôleur (non utilisé)
 * @return CallbackReturn::SUCCESS
 * @note Mesure de sécurité : arrêt immédiat de tous les mouvements
 */
controller_interface::CallbackReturn GripperExampleControllerSimu::on_deactivate(
    const rclcpp_lifecycle::State&) {
  // Arrêt de sécurité : vitesse à zéro pour les deux doigts
  if (command_interfaces_.size() >= 2) {
    command_interfaces_[0].set_value(0.0); // Doigt 1 - Arrêt
    command_interfaces_[1].set_value(0.0); // Doigt 2 - Arrêt
    RCLCPP_DEBUG(get_node()->get_logger(), 
                "Both gripper velocities set to zero during deactivation");
  }
  
  return CallbackReturn::SUCCESS;
}

/**
 * @brief Boucle principale de contrôle - exécutée à chaque cycle
 * 
 * Cette méthode implémente la logique de contrôle principal du contrôleur.
 * Elle gère les deux modes de contrôle (vitesse et position) et applique
 * les commandes appropriées aux interfaces de commande.
 * 
 * @param time Temps actuel (non utilisé)
 * @param period Période d'exécution (non utilisé)
 * @return return_type::OK si le cycle s'exécute correctement
 * 
 * @note Modes de contrôle :
 *       - Mode 0 (vitesse) : Application directe de la vitesse avec direction
 *       - Mode 1 (position) : Contrôle proportionnel avec limitation de vitesse
 */
controller_interface::return_type GripperExampleControllerSimu::update(const rclcpp::Time&,
                                                                  const rclcpp::Duration&) {
  // Vérification de la disponibilité des interfaces de commande
  if (command_interfaces_.size() >= 2) {
    
    if (control_mode_ == 0) {
      // ===== MODE CONTRÔLE EN VITESSE =====
      // Application directe de la vitesse avec direction aux deux doigts
      command_interfaces_[0].set_value(current_velocity_); // Doigt 1
      command_interfaces_[1].set_value(current_velocity_); // Doigt 2
      
    } else if (control_mode_ == 1) {
      // ===== MODE CONTRÔLE EN POSITION =====
      
      // Calcul de la position moyenne actuelle
      double avg_position = (finger1_position_ + finger2_position_) / 2.0;
      
      // Calcul de l'erreur de position
      double error = target_position_ - avg_position;
      
      // Contrôleur proportionnel : vitesse = gain * erreur
      double velocity = position_gain_ * error;
      
      // Limitation de vitesse pour la sécurité
      const double max_safe_velocity = 0.5;  // Vitesse maximale sécurisée
      if (velocity > max_safe_velocity) {
        velocity = max_safe_velocity;
      } else if (velocity < -max_safe_velocity) {
        velocity = -max_safe_velocity;
      }
      
      // Application de la vitesse calculée aux deux doigts
      command_interfaces_[0].set_value(velocity); // Doigt 1
      command_interfaces_[1].set_value(velocity); // Doigt 2
      
      // Logging des informations de contrôle en position
      RCLCPP_DEBUG(get_node()->get_logger(), 
                  "Position control: target=%f, current=%f, error=%f, velocity=%f",
                  target_position_, avg_position, error, velocity);
      
      // Arrêt fin de course : si la position cible est atteinte
      if (std::abs(error) < position_threshold_) {
        command_interfaces_[0].set_value(0.0); // Arrêt doigt 1
        command_interfaces_[1].set_value(0.0); // Arrêt doigt 2
        RCLCPP_DEBUG(get_node()->get_logger(), "Position reached, stopping gripper");
      }
    }
  }
  
  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

// Export du plugin pour ROS2 Control
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::GripperExampleControllerSimu,
                       controller_interface::ControllerInterface)