// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file cartesian_velocity_example_controller.cpp
 * @brief Contrôleur de vitesse cartésienne pour robot Franka FR3
 * @details Ce contrôleur permet de commander le robot Franka FR3 en vitesse cartésienne
 *          via des commandes /cmd_vel. Il est conçu pour la téléopération avec un
 *          contrôleur haptique Desktop 6D de Haption dans un environnement ROS2 Humble.
 * @author Vincent Bassemayousse
 * @date 07/10/2025
 * @version 1.0
 */

#include <franka_example_controllers/cartesian_velocity_example_controller.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <mutex>
#include <algorithm>

#include <Eigen/Eigen>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace franka_example_controllers {

// ==================== CONSTANTES DE SÉCURITÉ ====================
/**
 * @brief Vitesse linéaire maximale autorisée pour le robot
 * @details Limite de sécurité pour éviter les mouvements trop rapides
 *          qui pourraient endommager le robot ou être dangereux
 */
constexpr double MAX_LINEAR_VELOCITY = 0.5;   // m/s

/**
 * @brief Vitesse angulaire maximale autorisée pour le robot
 * @details Limite de sécurité pour les rotations de l'effecteur
 */
constexpr double MAX_ANGULAR_VELOCITY = 0.75;  // rad/s

/**
 * @brief Facteur de lissage pour les transitions de vitesse
 * @details Utilisé pour éviter les discontinuités dans les commandes
 *          de vitesse et assurer des mouvements fluides
 */
constexpr double VELOCITY_SMOOTHING_FACTOR = 0.01; // Facteur de lissage

// ==================== CONFIGURATION DES INTERFACES ====================

/**
 * @brief Configuration des interfaces de commande
 * @details Définit les interfaces nécessaires pour commander le robot
 *          en vitesse cartésienne. Utilise l'interface de vitesse cartésienne
 *          fournie par franka_semantic_components.
 * @return Configuration des interfaces de commande
 */
controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Récupération des noms des interfaces de commande de vitesse cartésienne
  config.names = franka_cartesian_velocity_->get_command_interface_names();

  return config;
}

/**
 * @brief Configuration des interfaces d'état
 * @details Ce contrôleur n'a pas besoin d'interfaces d'état spécifiques
 *          car il fonctionne uniquement en commande de vitesse
 * @return Configuration vide pour les interfaces d'état
 */
controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

// ==================== FONCTIONS UTILITAIRES ====================

/**
 * @brief Limite et valide un vecteur de vitesse
 * @details Vérifie que les composantes du vecteur sont finies (pas NaN ou inf)
 *          et limite la norme du vecteur à la valeur maximale spécifiée
 * @param velocity Vecteur de vitesse à limiter
 * @param max_velocity Vitesse maximale autorisée (norme du vecteur)
 * @return Vecteur de vitesse limité et validé
 */
Eigen::Vector3d CartesianVelocityExampleController::limitVelocity(
    const Eigen::Vector3d& velocity, double max_velocity) {
  Eigen::Vector3d limited_velocity = velocity;
  
  // Vérification des valeurs invalides (NaN, inf) pour chaque composante
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(limited_velocity(i))) {
      limited_velocity(i) = 0.0;
      RCLCPP_WARN(get_node()->get_logger(), 
                  "Invalid velocity component [%d] detected, set to 0.0", i);
    }
  }
  
  // Limitation de la norme du vecteur vitesse
  double velocity_norm = limited_velocity.norm();
  if (velocity_norm > max_velocity) {
    // Mise à l'échelle proportionnelle pour maintenir la direction
    limited_velocity = limited_velocity * (max_velocity / velocity_norm);
  }
  
  return limited_velocity;
}

/**
 * @brief Applique un lissage aux transitions de vitesse
 * @details Utilise un filtre passe-bas simple pour éviter les discontinuités
 *          dans les commandes de vitesse, assurant des mouvements fluides
 * @param target_velocity Vitesse cible désirée
 * @param current_velocity Vitesse actuelle du robot
 * @return Nouvelle vitesse lissée
 */
Eigen::Vector3d CartesianVelocityExampleController::smoothVelocity(
    const Eigen::Vector3d& target_velocity, const Eigen::Vector3d& current_velocity) {
  // Filtre passe-bas simple : v_new = v_old + α * (v_target - v_old)
  return current_velocity + VELOCITY_SMOOTHING_FACTOR * (target_velocity - current_velocity);
}

// ==================== BOUCLE DE CONTRÔLE PRINCIPALE ====================

/**
 * @brief Boucle de contrôle principale du contrôleur
 * @details Appelée à chaque itération du contrôleur (généralement à 1000 Hz).
 *          Récupère les commandes de vitesse, applique les limites de sécurité,
 *          effectue le lissage et envoie les commandes au robot.
 * @param time Temps actuel (non utilisé dans ce contrôleur)
 * @param period Période d'échantillonnage (non utilisée dans ce contrôleur)
 * @return État de retour du contrôleur (OK ou ERROR)
 */
controller_interface::return_type CartesianVelocityExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  
  // ===== RÉCUPÉRATION THREAD-SAFE DES COMMANDES =====
  Eigen::Vector3d target_linear_velocity;
  Eigen::Vector3d target_angular_velocity;
  
  {
    // Verrouillage pour accès thread-safe aux variables partagées
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    target_linear_velocity = target_linear_velocity_;
    target_angular_velocity = target_angular_velocity_;
  }

  // ===== APPLICATION DES LIMITES DE SÉCURITÉ =====
  target_linear_velocity = limitVelocity(target_linear_velocity, MAX_LINEAR_VELOCITY);
  target_angular_velocity = limitVelocity(target_angular_velocity, MAX_ANGULAR_VELOCITY);
  
  // ===== LISSAGE DES TRANSITIONS =====
  // Évite les discontinuités qui pourraient causer des secousses
  current_linear_velocity_ = smoothVelocity(target_linear_velocity, current_linear_velocity_);
  current_angular_velocity_ = smoothVelocity(target_angular_velocity, current_angular_velocity_);

  // ===== ENVOI DE LA COMMANDE AU ROBOT =====
  if (franka_cartesian_velocity_->setCommand(current_linear_velocity_,
                                             current_angular_velocity_)) {
    return controller_interface::return_type::OK;
  } else {
    // Erreur critique : impossible d'envoyer la commande
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

// ==================== CALLBACK DE RÉCEPTION DES COMMANDES ====================

/**
 * @brief Callback pour les messages de commande de vitesse
 * @details Reçoit les commandes de vitesse depuis le topic /cmd_vel,
 *          valide le format et met à jour les vitesses cibles.
 *          Format attendu : [v_x, v_y, v_z, ω_x, ω_y, ω_z, trigger]
 * @param msg Message contenant les 7 composantes de vitesse
 * @note Les composantes Y et Z sont inversées pour correspondre
 *       au repère du contrôleur haptique Desktop 6D
 */
void CartesianVelocityExampleController::cmd_vel_callback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  
  // ===== VALIDATION DU FORMAT DU MESSAGE =====
  if (msg->data.size() != 7) {
    RCLCPP_WARN(get_node()->get_logger(), 
                "Received cmd_vel with incorrect size: %zu, expected 6", 
                msg->data.size());
    return;
  }

  // ===== MISE À JOUR THREAD-SAFE DES VITESSES CIBLES =====
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    
    // Extraction des vitesses linéaires (x, y, z)
    // Note : Inversion des axes Y et Z pour correspondre au repère haptique
    target_linear_velocity_(0) = msg->data[0];   // v_x (avant/arrière)
    target_linear_velocity_(1) = -msg->data[1];  // v_y (gauche/droite - inversé)
    target_linear_velocity_(2) = -msg->data[2];  // v_z (haut/bas - inversé)
    
    // Extraction des vitesses angulaires (rx, ry, rz)
    // Note : Inversion des axes Y et Z pour correspondre au repère haptique
    target_angular_velocity_(0) = msg->data[3];   // ω_x (roulis)
    target_angular_velocity_(1) = -msg->data[4];  // ω_y (tangage - inversé)
    target_angular_velocity_(2) = -msg->data[5];  // ω_z (lacet - inversé)
  }

  // ===== LOGGING DE DEBUG =====
  RCLCPP_DEBUG(get_node()->get_logger(), 
               "Received cmd_vel: linear=[%.3f, %.3f, %.3f], angular=[%.3f, %.3f, %.3f]",
               target_linear_velocity_(0), target_linear_velocity_(1), target_linear_velocity_(2),
               target_angular_velocity_(0), target_angular_velocity_(1), target_angular_velocity_(2));
}

// ==================== CALLBACKS DU CYCLE DE VIE ====================

/**
 * @brief Initialisation du contrôleur
 * @details Première étape du cycle de vie du contrôleur.
 *          Aucune initialisation spécifique n'est nécessaire ici.
 * @return SUCCESS si l'initialisation s'est bien déroulée
 */
controller_interface::CallbackReturn CartesianVelocityExampleController::on_init() {
  RCLCPP_INFO(get_node()->get_logger(), "Cartesian velocity controller initialized");
  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Configuration du contrôleur
 * @details Deuxième étape du cycle de vie. Configure les interfaces,
 *          crée les subscribers, initialise les variables et configure
 *          le comportement de collision du robot.
 * @param previous_state État précédent du contrôleur
 * @return SUCCESS si la configuration s'est bien déroulée
 */
controller_interface::CallbackReturn CartesianVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // ===== CRÉATION DE L'INTERFACE DE VITESSE CARTÉSIENNE =====
  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(
          franka_semantic_components::FrankaCartesianVelocityInterface(k_elbow_activated_));

  // ===== CONFIGURATION DU SUBSCRIBER =====
  // Configuration QoS pour une réception fiable des commandes
  rclcpp::QoS qos(10);
  qos.reliability(rclcpp::ReliabilityPolicy::Reliable);  // Livraison garantie
  qos.durability(rclcpp::DurabilityPolicy::Volatile);    // Pas de persistance
  
  // Création du subscriber pour les commandes de vitesse
  cmd_vel_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/cmd_vel",
      qos,
      std::bind(&CartesianVelocityExampleController::cmd_vel_callback, this, std::placeholders::_1));

  // ===== INITIALISATION DES VARIABLES =====
  current_linear_velocity_.setZero();
  current_angular_velocity_.setZero();
  target_linear_velocity_.setZero();
  target_angular_velocity_.setZero();

  // ===== CONFIGURATION DU COMPORTEMENT DE COLLISION =====
  try {
    // Création du client pour le service de configuration de collision
    auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
        "service_server/set_full_collision_behavior");
    
    // Attente du service avec timeout
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(get_node()->get_logger(), 
                  "Collision behavior service not available, continuing without setting it");
    } else {
      // Récupération des paramètres de collision par défaut
      auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
      auto future_result = client->async_send_request(request);
      
      // Attente non-bloquante de la réponse
      if (future_result.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        auto success = future_result.get();
        if (success) {
          RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
        } else {
          RCLCPP_WARN(get_node()->get_logger(), "Failed to set default collision behavior.");
        }
      } else {
        RCLCPP_WARN(get_node()->get_logger(), "Timeout setting collision behavior.");
      }
    }
  } catch (const std::exception& e) {
    // Gestion des exceptions lors de la configuration de collision
    RCLCPP_WARN(get_node()->get_logger(), 
                "Exception while setting collision behavior: %s", e.what());
  }

  // ===== MESSAGES D'INFORMATION =====
  RCLCPP_INFO(get_node()->get_logger(), 
              "Cartesian velocity controller configured. Listening to /cmd_vel");
  RCLCPP_INFO(get_node()->get_logger(), 
              "Velocity limits: linear=%.2f m/s, angular=%.2f rad/s", 
              MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  RCLCPP_INFO(get_node()->get_logger(), 
              "Smoothing factor: %.3f", VELOCITY_SMOOTHING_FACTOR);
  
  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Activation du contrôleur
 * @details Troisième étape du cycle de vie. Associe les interfaces
 *          de commande et réinitialise les vitesses à zéro.
 * @param previous_state État précédent du contrôleur
 * @return SUCCESS si l'activation s'est bien déroulée
 */
controller_interface::CallbackReturn CartesianVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // ===== ASSOCIATION DES INTERFACES DE COMMANDE =====
  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);
  
  // ===== RÉINITIALISATION DES VITESSES =====
  // Important : assure un démarrage propre sans mouvement résiduel
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    current_linear_velocity_.setZero();
    current_angular_velocity_.setZero();
    target_linear_velocity_.setZero();
    target_angular_velocity_.setZero();
  }
  
  RCLCPP_INFO(get_node()->get_logger(), 
              "Cartesian velocity controller activated - Ready for teleoperation");
  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief Désactivation du contrôleur
 * @details Quatrième étape du cycle de vie. Libère les interfaces
 *          et arrête le robot en remettant toutes les vitesses à zéro.
 * @param previous_state État précédent du contrôleur
 * @return SUCCESS si la désactivation s'est bien déroulée
 */
controller_interface::CallbackReturn CartesianVelocityExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // ===== LIBÉRATION DES INTERFACES =====
  franka_cartesian_velocity_->release_interfaces();
  
  // ===== ARRÊT SÉCURISÉ DU ROBOT =====
  // Importante mesure de sécurité : arrêt complet du mouvement
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    current_linear_velocity_.setZero();
    current_angular_velocity_.setZero();
    target_linear_velocity_.setZero();
    target_angular_velocity_.setZero();
  }
  
  RCLCPP_INFO(get_node()->get_logger(), 
              "Cartesian velocity controller deactivated - Robot stopped");
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers

// ==================== DÉCLARATION DU PLUGIN ====================
/**
 * @brief Déclaration du plugin pour le système de contrôleurs ROS2
 * @details Permet au système de contrôleurs ROS2 de charger dynamiquement
 *          ce contrôleur comme un plugin
 */
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerInterface)