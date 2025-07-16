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
 * @file joint_position_example_controller.cpp
 * @brief Contrôleur de position des articulations pour robot Franka FR3 avec téléopération haptique
 * 
 * Ce contrôleur implémente un système de contrôle de position des articulations avec lissage
 * de trajectoire pour la téléopération d'un robot Franka FR3 via un contrôleur haptique
 * Desktop 6D de Haption. Il supporte à la fois la simulation Gazebo et le robot réel.
 * 
 * Fonctionnalités principales :
 * - Contrôle de position des 7 articulations du robot Franka
 * - Lissage de trajectoire avec filtre passe-bas
 * - Limitation de vitesse et d'accélération pour la sécurité
 * - Support simulation Gazebo et robot réel
 * - Communication thread-safe via ROS2 topics
 * 
 * @author Vincent Bassemayousse
 * @date 07/10/2025
 * @version 1.0
 */

#include <franka_example_controllers/joint_position_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <algorithm>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <mutex>

namespace franka_example_controllers {

/**
 * @brief Configuration des interfaces de commande pour le contrôleur
 * 
 * Cette méthode définit quelles interfaces de commande le contrôleur va utiliser.
 * Pour chaque articulation du robot, elle configure l'interface de position correspondante.
 * 
 * @return controller_interface::InterfaceConfiguration Configuration des interfaces de commande
 * 
 * @details
 * - Type: INDIVIDUAL (chaque interface est configurée individuellement)
 * - Interfaces: position de chaque articulation (arm_id_joint1/position, arm_id_joint2/position, etc.)
 * - Utilisé par le framework ROS2 Control pour établir les connexions avec le hardware
 */
controller_interface::InterfaceConfiguration
JointPositionExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Configuration des interfaces de position pour chaque articulation
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  
  return config;
}

/**
 * @brief Configuration des interfaces d'état pour le contrôleur
 * 
 * Cette méthode définit quelles interfaces d'état le contrôleur va lire.
 * Elle configure les interfaces de position pour chaque articulation et,
 * si ce n'est pas Gazebo, ajoute l'interface de temps robot.
 * 
 * @return controller_interface::InterfaceConfiguration Configuration des interfaces d'état
 * 
 * @details
 * - Type: INDIVIDUAL (chaque interface est configurée individuellement)
 * - Interfaces: position de chaque articulation pour la rétroaction
 * - Interface temps robot: uniquement pour le robot réel (pas Gazebo)
 * - Utilisé pour lire l'état actuel du robot et calculer les commandes
 */
controller_interface::InterfaceConfiguration
JointPositionExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Configuration des interfaces de position pour chaque articulation
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  
  // Ajout de l'interface temps robot uniquement pour le robot réel
  if (!is_gazebo_) {
    config.names.push_back(arm_id_ + "/robot_time");
  }
  
  return config;
}

/**
 * @brief Boucle principale de contrôle - appelée à chaque cycle de contrôle
 * 
 * Cette méthode est le cœur du contrôleur. Elle est appelée périodiquement
 * par le framework ROS2 Control et implémente la logique de contrôle principale.
 * 
 * @param time Temps ROS2 actuel (non utilisé dans cette implémentation)
 * @param period Période écoulée depuis le dernier appel (non utilisé dans cette implémentation)
 * @return controller_interface::return_type::OK si le contrôle s'est bien passé
 * 
 * @details
 * Séquence d'exécution:
 * 1. Initialisation au premier appel (lecture positions initiales)
 * 2. Calcul du temps écoulé (différent selon Gazebo/robot réel)
 * 3. Lecture thread-safe des commandes externes
 * 4. Application du lissage de trajectoire si nouvelle commande
 * 5. Envoi des commandes de position aux articulations
 * 
 * @note Cette méthode doit être rapide car elle est appelée à haute fréquence (1kHz typiquement)
 */
controller_interface::return_type JointPositionExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  // === PHASE D'INITIALISATION ===
  // Exécutée uniquement au premier appel pour initialiser les positions
  if (initialization_flag_) {
    // Lecture des positions initiales de chaque articulation
    for (int i = 0; i < num_joints; ++i) {
      initial_q_.at(i) = state_interfaces_[i].get_value();           // Position initiale
      target_positions_.at(i) = initial_q_.at(i);                   // Position cible lissée
      target_positions_raw_.at(i) = initial_q_.at(i);               // Position cible brute
      previous_positions_.at(i) = initial_q_.at(i);                 // Position précédente
      current_velocities_.at(i) = 0.0;                              // Vitesse initiale nulle
    }
    initialization_flag_ = false;
    
    // Initialisation du temps robot (uniquement pour robot réel)
    if (!is_gazebo_) {
      initial_robot_time_ = state_interfaces_.back().get_value();
    }
    elapsed_time_ = 0.0;
  } else {
    // === CALCUL DU TEMPS ÉCOULÉ ===
    // Méthode différente selon l'environnement (Gazebo vs robot réel)
    if (!is_gazebo_) {
      // Robot réel: utilise l'horloge robot
      robot_time_ = state_interfaces_.back().get_value();
      elapsed_time_ = robot_time_ - initial_robot_time_;
    } else {
      // Gazebo: incrémente avec la période de trajectoire
      elapsed_time_ += trajectory_period_;
    }
  }

  // === CALCUL DU PAS DE TEMPS POUR LE LISSAGE ===
  // Période de contrôle différente selon l'environnement
  double dt = is_gazebo_ ? trajectory_period_ : 0.001; // 1ms typique pour robot réel

  // === LECTURE THREAD-SAFE DES COMMANDES EXTERNES ===
  // Protection par mutex pour éviter les conditions de course
  std_msgs::msg::Float32MultiArray::SharedPtr cmd;
  bool new_cmd_received = false;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd = last_joint_positions_;
    new_cmd_received = new_command_received_;
    new_command_received_ = false; // Reset du flag
  }

  // === TRAITEMENT DES COMMANDES EXTERNES ===
  if (use_external_targets_ && cmd && !cmd->data.empty()) {
    // Mise à jour des cibles brutes si nouvelle commande reçue
    if (new_cmd_received) {
      for (int i = 0; i < num_joints; i++) {
        if (i < static_cast<int>(cmd->data.size())) {
          target_positions_raw_[i] = cmd->data[i];
        }
      }
    }
    
    // === APPLICATION DU LISSAGE DE TRAJECTOIRE ===
    // Filtre passe-bas + limitation vitesse/accélération
    smoothTrajectory(dt);
    
    // === ENVOI DES COMMANDES AUX ARTICULATIONS ===
    // Application des positions lissées aux interfaces de commande
    for (int i = 0; i < num_joints; i++) {
      command_interfaces_[i].set_value(target_positions_[i]);
    }
    
    RCLCPP_DEBUG(get_node()->get_logger(), "Applied smoothed joint position commands");
  } else {
    // === MAINTIEN DE LA POSITION ACTUELLE ===
    // Si pas de commande externe, maintenir les dernières positions cibles
    for (int i = 0; i < num_joints; i++) {
      command_interfaces_[i].set_value(target_positions_[i]);
    }
    RCLCPP_DEBUG(get_node()->get_logger(), "Maintained current positions");
  }

  return controller_interface::return_type::OK;
}

/**
 * @brief Applique le lissage de trajectoire avec filtre passe-bas et limitation vitesse/accélération
 * 
 * Cette méthode implémente un algorithme de lissage de trajectoire en trois étapes:
 * 1. Filtre passe-bas pour réduire les variations brusques
 * 2. Limitation de vitesse et d'accélération
 * 3. Mise à jour des états de vitesse
 * 
 * @param dt Pas de temps depuis le dernier appel [s]
 * 
 * @details
 * Algorithme appliqué pour chaque articulation:
 * - Filtrage passe-bas: target_filtered = α*target_raw + (1-α)*target_previous
 * - Limitation vitesse: |Δpos| ≤ v_max * dt
 * - Limitation accélération: |Δvel| ≤ a_max * dt
 * 
 * @note Cette méthode modifie les variables membres:
 * - target_positions_: positions cibles lissées
 * - current_velocities_: vitesses actuelles calculées
 * - previous_positions_: positions précédentes sauvegardées
 */
void JointPositionExampleController::smoothTrajectory(double dt) {
  for (int i = 0; i < num_joints; ++i) {
    // === ÉTAPE 1: FILTRE PASSE-BAS ===
    // Lisse les variations brusques de la position cible
    double filtered_target = applyLowPassFilter(target_positions_raw_[i], 
                                               target_positions_[i], 
                                               smoothing_factor_);
    
    // === ÉTAPE 2: LIMITATION VITESSE ET ACCÉLÉRATION ===
    // Applique les contraintes cinématiques
    target_positions_[i] = limitVelocity(filtered_target, 
                                        target_positions_[i], 
                                        current_velocities_[i], 
                                        dt, i);
    
    // === ÉTAPE 3: MISE À JOUR DES ÉTATS ===
    // Calcul de la vitesse actuelle par différence finie
    current_velocities_[i] = (target_positions_[i] - previous_positions_[i]) / dt;
    
    // Sauvegarde pour le prochain cycle
    previous_positions_[i] = target_positions_[i];
  }
}

/**
 * @brief Limite la vitesse et l'accélération d'une articulation
 * 
 * Cette méthode implémente des contraintes cinématiques pour assurer un mouvement fluide
 * et sécurisé du robot. Elle limite séquentiellement la vitesse puis l'accélération.
 * 
 * @param desired_pos Position désirée [rad]
 * @param current_pos Position actuelle [rad]
 * @param current_vel Vitesse actuelle [rad/s]
 * @param dt Pas de temps [s]
 * @param joint_idx Index de l'articulation (pour debug futur)
 * @return double Position limitée respectant les contraintes [rad]
 * 
 * @details
 * Algorithme de limitation:
 * 1. Calcul de l'erreur de position: Δpos = pos_desired - pos_current
 * 2. Limitation vitesse: |Δpos| ≤ v_max * dt
 * 3. Calcul vitesse cible: v_target = Δpos_limited / dt
 * 4. Limitation accélération: |Δvel| ≤ a_max * dt
 * 5. Application: pos_new = pos_current + v_limited * dt
 * 
 * @note Les paramètres max_velocity_ et max_acceleration_ sont configurables
 */
double JointPositionExampleController::limitVelocity(double desired_pos, 
                                                    double current_pos, 
                                                    double current_vel, 
                                                    double dt, 
                                                    int joint_idx) {
  // === LIMITATION DE VITESSE ===
  // Calcul de l'erreur de position
  double position_error = desired_pos - current_pos;
  
  // Calcul du changement maximal de position basé sur la vitesse limite
  double max_position_change = max_velocity_ * dt;
  
  // Application de la limitation de vitesse
  if (std::abs(position_error) > max_position_change) {
    position_error = std::copysign(max_position_change, position_error);
  }
  
  // === LIMITATION D'ACCÉLÉRATION ===
  // Calcul de la vitesse cible
  double target_velocity = position_error / dt;
  
  // Calcul du changement de vitesse requis
  double velocity_change = target_velocity - current_vel;
  
  // Calcul du changement maximal de vitesse basé sur l'accélération limite
  double max_velocity_change = max_acceleration_ * dt;
  
  // Application de la limitation d'accélération
  if (std::abs(velocity_change) > max_velocity_change) {
    velocity_change = std::copysign(max_velocity_change, velocity_change);
  }
  
  // === CALCUL DE LA POSITION FINALE ===
  // Calcul de la vitesse limitée
  double limited_velocity = current_vel + velocity_change;
  
  // Calcul de la nouvelle position
  return current_pos + limited_velocity * dt;
}

/**
 * @brief Applique un filtre passe-bas du premier ordre
 * 
 * Cette méthode implémente un filtre passe-bas simple pour lisser les signaux
 * et réduire les variations hautes fréquences qui pourraient causer des 
 * vibrations ou des mouvements brusques.
 * 
 * @param new_value Nouvelle valeur à filtrer
 * @param old_value Valeur précédente filtrée
 * @param alpha Facteur de lissage [0,1] (0=pas de lissage, 1=lissage maximal)
 * @return double Valeur filtrée
 * 
 * @details
 * Équation du filtre: y[n] = α * x[n] + (1-α) * y[n-1]
 * - α proche de 0: lissage fort, réponse lente
 * - α proche de 1: lissage faible, réponse rapide
 * 
 * @note Le paramètre alpha correspond à smoothing_factor_ dans le code
 */
double JointPositionExampleController::applyLowPassFilter(double new_value, 
                                                         double old_value, 
                                                         double alpha) {
  return alpha * new_value + (1.0 - alpha) * old_value;
}

/**
 * @brief Initialisation du contrôleur - déclaration des paramètres
 * 
 * Cette méthode est appelée lors de l'initialisation du contrôleur.
 * Elle déclare tous les paramètres configurables avec leurs valeurs par défaut.
 * 
 * @return CallbackReturn::SUCCESS si l'initialisation s'est bien passée
 * @return CallbackReturn::ERROR en cas d'exception
 * 
 * @details
 * Paramètres déclarés:
 * - gazebo: Mode simulation (false = robot réel)
 * - robot_description: Description URDF du robot
 * - use_external_targets: Utilisation de commandes externes via topic
 * - max_velocity: Vitesse maximale autorisée [rad/s]
 * - max_acceleration: Accélération maximale autorisée [rad/s²]
 * - smoothing_factor: Facteur de lissage du filtre passe-bas [0,1]
 * 
 * @note Cette méthode ne fait que déclarer les paramètres, leur lecture
 *       se fait dans on_configure()
 */
CallbackReturn JointPositionExampleController::on_init() {
  try {
    // Déclaration des paramètres avec valeurs par défaut
    auto_declare<bool>("gazebo", false);                    // Mode simulation
    auto_declare<std::string>("robot_description", "");     // Description URDF
    auto_declare<bool>("use_external_targets", true);       // Commandes externes
    auto_declare<double>("max_velocity", 0.1);              // Vitesse max [rad/s]
    auto_declare<double>("max_acceleration", 0.5);          // Accélération max [rad/s²]
    auto_declare<double>("smoothing_factor", 0.01);         // Facteur de lissage
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

/**
 * @brief Configuration du contrôleur - lecture des paramètres et initialisation
 * 
 * Cette méthode est appelée après on_init() et configure le contrôleur
 * avec les paramètres lus depuis le serveur de paramètres ROS2.
 * 
 * @param previous_state État précédent du contrôleur (non utilisé)
 * @return CallbackReturn::SUCCESS si la configuration s'est bien passée
 * 
 * @details
 * Étapes de configuration:
 * 1. Lecture des paramètres depuis le serveur ROS2
 * 2. Récupération de la description URDF du robot
 * 3. Extraction du nom du robot depuis l'URDF
 * 4. Initialisation des vecteurs de données
 * 5. Création du subscriber pour les commandes externes (si activé)
 * 
 * @note Cette méthode prépare toutes les structures de données nécessaires
 *       au fonctionnement du contrôleur
 */
CallbackReturn JointPositionExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // === LECTURE DES PARAMÈTRES ===
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();
  use_external_targets_ = get_node()->get_parameter("use_external_targets").as_bool();
  max_velocity_ = get_node()->get_parameter("max_velocity").as_double();
  max_acceleration_ = get_node()->get_parameter("max_acceleration").as_double();
  smoothing_factor_ = get_node()->get_parameter("smoothing_factor").as_double();

  // === RÉCUPÉRATION DE LA DESCRIPTION DU ROBOT ===
  // Connexion au serveur de paramètres pour récupérer l'URDF
  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  // Récupération asynchrone de la description URDF
  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  // === EXTRACTION DU NOM DU ROBOT ===
  // Analyse de l'URDF pour extraire le nom du robot (ex: "panda")
  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  // === INITIALISATION DES VECTEURS DE DONNÉES ===
  // Redimensionnement de tous les vecteurs avec la taille correcte
  target_positions_.resize(num_joints, 0.0);        // Positions cibles lissées
  target_positions_raw_.resize(num_joints, 0.0);    // Positions cibles brutes
  initial_q_.resize(num_joints, 0.0);               // Positions initiales
  current_velocities_.resize(num_joints, 0.0);      // Vitesses actuelles
  previous_positions_.resize(num_joints, 0.0);      // Positions précédentes

  // === CRÉATION DU SUBSCRIBER POUR COMMANDES EXTERNES ===
  if (use_external_targets_) {
    // Création du subscriber pour recevoir les commandes de position
    joint_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/joint_positions", 10, 
      std::bind(&JointPositionExampleController::commandCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_node()->get_logger(), 
                "JointPositionExampleController initialized with external target subscription to /joint_positions");
    RCLCPP_INFO(get_node()->get_logger(), 
                "Smoothing parameters: max_vel=%.2f, max_acc=%.2f, smoothing_factor=%.2f",
                max_velocity_, max_acceleration_, smoothing_factor_);
  } else {
    RCLCPP_INFO(get_node()->get_logger(), 
                "JointPositionExampleController initialized with default trajectory behavior");
  }

  return CallbackReturn::SUCCESS;
}

/**
 * @brief Activation du contrôleur - préparation avant démarrage
 * 
 * Cette méthode est appelée juste avant que le contrôleur ne commence
 * à recevoir des appels à update(). Elle remet à zéro tous les flags
 * et prépare le contrôleur pour un nouveau cycle de fonctionnement.
 * 
 * @param previous_state État précédent du contrôleur (non utilisé)
 * @return CallbackReturn::SUCCESS si l'activation s'est bien passée
 * 
 * @details
 * Actions d'activation:
 * - Activation du flag d'initialisation
 * - Remise à zéro du temps écoulé
 * - Remise à zéro du flag de nouvelle commande
 * 
 * @note Cette méthode est appelée chaque fois que le contrôleur passe
 *       de l'état "inactive" à l'état "active"
 */
CallbackReturn JointPositionExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // === RÉINITIALISATION DES FLAGS ===
  initialization_flag_ = true;          // Déclenche l'initialisation au premier update()
  elapsed_time_ = 0.0;                  // Remise à zéro du compteur de temps
  new_command_received_ = false;        // Pas de nouvelle commande en attente
  
  return CallbackReturn::SUCCESS;
}

/**
 * @brief Callback pour réception des commandes de position articulaire
 * 
 * Cette méthode est appelée chaque fois qu'un nouveau message de commande
 * de position est reçu sur le topic "/joint_positions". Elle stocke
 * la commande de manière thread-safe pour utilisation dans update().
 * 
 * @param msg Message contenant les positions cibles pour chaque articulation
 * 
 * @details
 * Validation et stockage:
 * 1. Vérification que le message contient assez de positions
 * 2. Stockage thread-safe avec protection par mutex
 * 3. Activation du flag de nouvelle commande
 * 
 * Format du message attendu:
 * - std_msgs::msg::Float32MultiArray
 * - data[0..6]: positions des 7 articulations [rad]
 * 
 * @note Cette méthode s'exécute dans un thread différent de update(),
 *       d'où la nécessité de la protection par mutex
 * 
 * @warning Si le message contient moins de positions que d'articulations,
 *          la commande est rejetée avec un message d'erreur
 */
void JointPositionExampleController::commandCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  
  // === VALIDATION DU MESSAGE ===
  // Vérification que le message contient assez de positions
  if (msg->data.size() < static_cast<size_t>(num_joints)) {
    RCLCPP_ERROR(get_node()->get_logger(), 
                "Received joint command with insufficient joint positions (%zu), expected %d",
                msg->data.size(), num_joints);
    return;
  }

  // === STOCKAGE THREAD-SAFE ===
  // Protection par mutex pour éviter les conditions de course avec update()
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    last_joint_positions_ = msg;        // Stockage du message
    new_command_received_ = true;       // Signalement de nouvelle commande
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "Received new joint position command");
}

}  // namespace franka_example_controllers

// === DÉCLARATION DU PLUGIN ===
// Macro pour exporter le contrôleur en tant que plugin ROS2 Control
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerInterface)