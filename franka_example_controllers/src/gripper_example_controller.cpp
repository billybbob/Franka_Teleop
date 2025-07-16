// Copyright (c) 2025 Franka Robotics GmbH
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
 * @file gripper_example_controller.cpp
 * @brief Contrôleur de pince pour robot Franka FR3 avec téléopération haptique
 * @author Vincent Bassemayousse
 * @date 07/10/2025
 * @version 1.0
 * 
 * Ce fichier implémente un contrôleur ROS2 pour la pince du robot Franka FR3 dans le cadre
 * d'un projet de téléopération utilisant un contrôleur haptique Desktop 6D de Haption.
 * Le contrôleur gère l'ouverture/fermeture de la pince basée sur les signaux de trigger
 * provenant du dispositif haptique.
 * 
 * Fonctionnalités principales :
 * - Contrôle de la pince via des signaux de trigger haptiques
 * - Actions ROS2 pour le mouvement et la prise d'objets
 * - Gestion des états de la pince (ouverte/fermée)
 * - Intégration avec la simulation Gazebo Fortress
 */

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "fmt/format.h"
#include "franka_example_controllers/default_robot_behavior_utils.hpp"
#include "franka_example_controllers/gripper_example_controller.hpp"

#include <std_msgs/msg/float32_multi_array.hpp>

// Définition des couleurs pour les messages de log
#define RED "\033[1;31m"      ///< Couleur rouge pour les messages d'erreur
#define GREEN "\033[1;32m"    ///< Couleur verte pour les messages de succès
#define YELLOW "\033[1;33m"   ///< Couleur jaune pour les messages d'avertissement
#define RESET "\033[0m"       ///< Reset de la couleur

namespace franka_example_controllers {

/**
 * @brief Configuration des interfaces de commande du contrôleur
 * 
 * Cette fonction définit les interfaces de commande requises par le contrôleur.
 * Dans ce cas, aucune interface de commande n'est nécessaire car le contrôleur
 * utilise uniquement des actions ROS2 pour communiquer avec la pince.
 * 
 * @return Configuration des interfaces de commande (NONE)
 */
controller_interface::InterfaceConfiguration
GripperExampleController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

/**
 * @brief Configuration des interfaces d'état du contrôleur
 * 
 * Cette fonction définit les interfaces d'état requises par le contrôleur.
 * Aucune interface d'état n'est nécessaire car le contrôleur ne lit pas
 * directement l'état du hardware.
 * 
 * @return Configuration des interfaces d'état (NONE)
 */
controller_interface::InterfaceConfiguration
GripperExampleController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

/**
 * @brief Initialisation du contrôleur
 * 
 * Cette fonction est appelée lors de la phase d'initialisation du contrôleur.
 * Elle déclare les paramètres nécessaires, notamment l'ID du bras robotique.
 * 
 * @return CallbackReturn::SUCCESS si l'initialisation réussit, CallbackReturn::ERROR sinon
 */
CallbackReturn GripperExampleController::on_init() {
  try {
    // Déclaration du paramètre arm_id avec une valeur par défaut "fr3"
    // Ce paramètre identifie le robot Franka FR3 utilisé
    auto_declare<std::string>("arm_id", "fr3");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

/**
 * @brief Configuration du contrôleur
 * 
 * Cette fonction configure tous les clients ROS2 nécessaires pour contrôler la pince :
 * - Client d'action pour la prise d'objets (grasp)
 * - Client d'action pour le mouvement de la pince (move)
 * - Client de service pour arrêter la pince (stop)
 * - Souscription au topic des commandes haptiques
 * 
 * @param state État actuel du lifecycle du contrôleur
 * @return CallbackReturn::SUCCESS si la configuration réussit, CallbackReturn::ERROR sinon
 */
CallbackReturn GripperExampleController::on_configure(const rclcpp_lifecycle::State&) {
  // Récupération du namespace du nœud pour construire les noms des topics/services
  namespace_ = get_node()->get_namespace();
  
  // Configuration du client d'action pour la prise d'objets
  // Ce client permet d'envoyer des commandes de fermeture avec force contrôlée
  gripper_grasp_action_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(
      get_node(), fmt::format("{}/franka_gripper/grasp", namespace_));

  // Configuration du client d'action pour le mouvement de la pince
  // Ce client permet d'envoyer des commandes d'ouverture/fermeture avec position contrôlée
  gripper_move_action_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(
      get_node(), fmt::format("{}/franka_gripper/move", namespace_));

  // Configuration du client de service pour arrêter la pince
  // Ce service permet d'arrêter toute action en cours sur la pince
  gripper_stop_client_ = get_node()->create_client<std_srvs::srv::Trigger>(
      fmt::format("{}/franka_gripper/stop", namespace_));

  // Configuration de la souscription au topic des commandes haptiques
  // Le topic "/valeur_effecteur" contient les données du contrôleur haptique Desktop 6D
  // incluant la position, orientation et les signaux de trigger
  pose_subscription_ = get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/valeur_effecteur", 10,
      std::bind(&GripperExampleController::poseCommandCallback, this, std::placeholders::_1));

  // Configuration des callbacks pour les actions
  assignMoveGoalOptionsCallbacks();
  assignGraspGoalOptionsCallbacks();
  
  // Vérification que tous les clients ont été créés avec succès
  return nullptr != gripper_grasp_action_client_ && nullptr != gripper_move_action_client_ &&
                 nullptr != gripper_stop_client_
             ? CallbackReturn::SUCCESS
             : CallbackReturn::ERROR;
}

/**
 * @brief Activation du contrôleur
 * 
 * Cette fonction active le contrôleur et attend que les serveurs d'action soient disponibles.
 * Elle initialise également l'état du trigger pour la détection des changements d'état.
 * 
 * @param state État actuel du lifecycle du contrôleur
 * @return CallbackReturn::SUCCESS si l'activation réussit, CallbackReturn::ERROR sinon
 */
CallbackReturn GripperExampleController::on_activate(const rclcpp_lifecycle::State&) {
  // Attente de la disponibilité du serveur d'action Move (ouverture/fermeture)
  if (!gripper_move_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Move Action server not available after waiting.");
    return CallbackReturn::ERROR;
  }
  
  // Attente de la disponibilité du serveur d'action Grasp (prise d'objets)
  if (!gripper_grasp_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Grasp Action server not available after waiting.");
    return CallbackReturn::ERROR;
  }
  
  // Initialisation de l'état du trigger haptique
  // Cette variable permet de détecter les changements d'état du trigger
  previous_trigger_state = false;
  
  // Information : le contrôleur est maintenant actif et attend les commandes de trigger
  RCLCPP_INFO(get_node()->get_logger(), "Gripper controller activated - waiting for trigger commands");
  
  return CallbackReturn::SUCCESS;
}

/**
 * @brief Désactivation du contrôleur
 * 
 * Cette fonction désactive le contrôleur et arrête toute action en cours sur la pince
 * pour assurer une désactivation sécurisée.
 * 
 * @param previous_state État précédent du lifecycle du contrôleur
 * @return CallbackReturn::SUCCESS toujours (désactivation réussie)
 */
controller_interface::CallbackReturn GripperExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // Vérification de la disponibilité du service d'arrêt
  if (gripper_stop_client_->service_is_ready()) {
    // Création de la requête pour arrêter la pince
    std_srvs::srv::Trigger::Request::SharedPtr request =
        std::make_shared<std_srvs::srv::Trigger::Request>();

    // Envoi de la commande d'arrêt de manière asynchrone
    auto result = gripper_stop_client_->async_send_request(request);
    if (result.get() && result.get()->success) {
      RCLCPP_INFO(get_node()->get_logger(), "Gripper stopped successfully.");
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to stop gripper.");
    }
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Gripper stop service is not available.");
  }
  return CallbackReturn::SUCCESS;
}

/**
 * @brief Fonction de mise à jour du contrôleur
 * 
 * Cette fonction est appelée périodiquement pendant l'exécution du contrôleur.
 * Dans cette implémentation, elle ne fait rien car tout le contrôle est basé
 * sur les événements (callbacks des messages haptiques).
 * 
 * @param time Temps actuel
 * @param period Période depuis la dernière mise à jour
 * @return controller_interface::return_type::OK toujours
 */
controller_interface::return_type GripperExampleController::update(const rclcpp::Time&,
                                                                   const rclcpp::Duration&) {
  return controller_interface::return_type::OK;
}

/**
 * @brief Callback pour les commandes de pose du contrôleur haptique
 * 
 * Cette fonction est appelée chaque fois qu'un message est reçu sur le topic
 * "/valeur_effecteur" contenant les données du contrôleur haptique Desktop 6D.
 * Elle extrait la valeur du trigger (index 7) et traite le contrôle de la pince.
 * 
 * Format attendu du message (Float32MultiArray) :
 * - Index 0-2 : Position (x, y, z) de l'effecteur haptique
 * - Index 3-6 : Orientation (quaternion) de l'effecteur haptique
 * - Index 7 : Valeur du trigger (0.0 = relâché, 1.0 = pressé)
 * 
 * @param msg Message contenant les données du contrôleur haptique
 */
void GripperExampleController::poseCommandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  // Vérification que le message contient suffisamment d'éléments
  // Le trigger est situé à l'index 7, donc nous avons besoin d'au moins 8 éléments
  if (msg->data.size() >= 8) {
    // Extraction de la valeur du trigger depuis le message haptique
    trigger = msg->data[7];
    RCLCPP_DEBUG(get_node()->get_logger(), "Trigger value: %f", trigger);
    
    // Traitement du contrôle basé sur l'état du trigger
    processTriggerControl();
    
  } else {
    // Avertissement si le message ne contient pas suffisamment d'éléments
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                         "valeur_effecteur array has only %zu elements, need at least 8 for gripper control", 
                         msg->data.size());
  }
}

/**
 * @brief Traitement du contrôle par trigger haptique
 * 
 * Cette fonction analyse l'état du trigger et détecte les changements d'état
 * pour déclencher l'ouverture ou la fermeture de la pince. Elle utilise un seuil
 * de 0.5 pour déterminer si le trigger est activé ou non.
 * 
 * Logique de contrôle :
 * - Trigger > 0.5 : Fermeture de la pince (grasp)
 * - Trigger <= 0.5 : Ouverture de la pince (move)
 * 
 * La fonction ne réagit qu'aux changements d'état pour éviter les commandes répétées.
 */
void GripperExampleController::processTriggerControl() {
  // Détermination de l'état actuel du trigger avec un seuil de 0.5
  bool current_trigger_state = (trigger > 0.5);
  
  // Détection des changements d'état du trigger
  if (current_trigger_state != previous_trigger_state) {
    if (current_trigger_state) {
      // Trigger activé : fermeture de la pince avec force contrôlée
      RCLCPP_INFO(get_node()->get_logger(), "Trigger activated (%.2f > 0.5) - Closing gripper", trigger);
      graspGripper();
    } else {
      // Trigger désactivé : ouverture de la pince
      RCLCPP_INFO(get_node()->get_logger(), "Trigger deactivated (%.2f <= 0.5) - Opening gripper", trigger);
      openGripper();
    }
    
    // Mise à jour de l'état précédent pour la prochaine comparaison
    previous_trigger_state = current_trigger_state;
  }
}

/**
 * @brief Configuration des callbacks pour les actions de mouvement
 * 
 * Cette fonction configure les callbacks pour les actions de type Move
 * (ouverture/fermeture de la pince avec contrôle de position).
 * 
 * Callbacks configurés :
 * - goal_response_callback : Réponse à la soumission d'un objectif
 * - feedback_callback : Retour d'information pendant l'exécution
 * - result_callback : Résultat final de l'action
 */
void GripperExampleController::assignMoveGoalOptionsCallbacks() {
  // Callback appelé lors de la réponse à la soumission d'un objectif Move
  move_goal_options_.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&
                 goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       RED "Move Goal (i.e. open gripper) NOT accepted." RESET);
        } else {
          RCLCPP_DEBUG(get_node()->get_logger(), "Move Goal accepted");
        }
      };

  // Callback appelé périodiquement pendant l'exécution de l'action Move
  // Fournit des informations sur l'état actuel de la pince
  move_goal_options_.feedback_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&,
             const std::shared_ptr<const franka_msgs::action::Move_Feedback>& feedback) {
        RCLCPP_DEBUG(get_node()->get_logger(), "Move Goal current_width [%f].",
                    feedback->current_width);
      };

  // Callback appelé à la fin de l'exécution de l'action Move
  // Indique si l'action a réussi ou échoué
  move_goal_options_.result_callback =
      [this](
          const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::WrappedResult& result) {
        RCLCPP_INFO(get_node()->get_logger(), "Move Goal result %s.",
                    (rclcpp_action::ResultCode::SUCCEEDED == result.code ? YELLOW "SUCCESS" RESET
                                                                         : RED "FAIL" RESET));
      };
}

/**
 * @brief Configuration des callbacks pour les actions de prise
 * 
 * Cette fonction configure les callbacks pour les actions de type Grasp
 * (fermeture de la pince avec contrôle de force).
 * 
 * Callbacks configurés :
 * - goal_response_callback : Réponse à la soumission d'un objectif
 * - feedback_callback : Retour d'information pendant l'exécution
 * - result_callback : Résultat final de l'action
 */
void GripperExampleController::assignGraspGoalOptionsCallbacks() {
  // Callback appelé lors de la réponse à la soumission d'un objectif Grasp
  grasp_goal_options_.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>&
                 goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_node()->get_logger(), RED "Grasp Goal NOT accepted." RESET);
        } else {
          RCLCPP_DEBUG(get_node()->get_logger(), "Grasp Goal accepted.");
        }
      };

  // Callback appelé périodiquement pendant l'exécution de l'action Grasp
  // Fournit des informations sur l'état actuel de la pince
  grasp_goal_options_.feedback_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>&,
             const std::shared_ptr<const franka_msgs::action::Grasp_Feedback>& feedback) {
        RCLCPP_DEBUG(get_node()->get_logger(), "Grasp Goal current_width: %f",
                    feedback->current_width);
      };

  // Callback appelé à la fin de l'exécution de l'action Grasp
  // Indique si l'action a réussi ou échoué
  grasp_goal_options_.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::WrappedResult&
                 result) {
        RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal result %s.",
                    (rclcpp_action::ResultCode::SUCCEEDED == result.code ? GREEN "SUCCESS" RESET
                                                                         : RED "FAIL" RESET));
      };
}

/**
 * @brief Fonction de basculement de l'état de la pince (legacy)
 * 
 * Cette fonction était utilisée dans la version précédente pour basculer
 * automatiquement entre les états ouvert/fermé. Elle n'est plus utilisée
 * dans la version actuelle qui utilise le contrôle par trigger, mais est
 * conservée pour la compatibilité.
 * 
 * @deprecated Utilisez processTriggerControl() à la place
 */
void GripperExampleController::toggleGripperState() {
  /*
   * Basculement simple entre les états ouvert et fermé de la pince
   */
  enum ordered_state { open, closed };
  static ordered_state ordered_gripper_state = ordered_state::closed;

  if (ordered_gripper_state == ordered_state::closed) {
    openGripper();
    ordered_gripper_state = ordered_state::open;
  } else {
    graspGripper();
    ordered_gripper_state = ordered_state::closed;
  }
}

/**
 * @brief Ouverture de la pince
 * 
 * Cette fonction envoie une commande d'ouverture de la pince en utilisant
 * l'action Move avec des paramètres prédéfinis.
 * 
 * Paramètres d'ouverture :
 * - Largeur : 0.08 m (80 mm) - ouverture maximale
 * - Vitesse : 0.2 m/s - vitesse d'ouverture modérée
 * 
 * @return true si la commande a été envoyée avec succès, false sinon
 */
bool GripperExampleController::openGripper() {
  RCLCPP_DEBUG(get_node()->get_logger(), "Opening the gripper - Submitting a Move Goal");

  // Définition de l'objectif d'ouverture de la pince
  franka_msgs::action::Move::Goal move_goal;
  move_goal.width = 0.08;  // Largeur d'ouverture : 80 mm
  move_goal.speed = 0.2;   // Vitesse d'ouverture : 0.2 m/s

  // Envoi asynchrone de l'objectif d'ouverture
  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>>
      move_goal_handle =
          gripper_move_action_client_->async_send_goal(move_goal, move_goal_options_);
  
  // Vérification de la validité de l'envoi
  bool ret = move_goal_handle.valid();
  if (ret) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Submitted a Move Goal");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), RED "Failed to submit a Move Goal" RESET);
  }
  return ret;
}

/**
 * @brief Fermeture de la pince avec contrôle de force
 * 
 * Cette fonction envoie une commande de fermeture de la pince en utilisant
 * l'action Grasp avec des paramètres adaptés à la saisie d'objets cylindriques
 * (marqueur, stylo, etc.).
 * 
 * Paramètres de prise :
 * - Largeur cible : 0.030 m (30 mm) - adaptée aux objets cylindriques
 * - Vitesse : 0.05 m/s - vitesse lente pour une prise précise
 * - Force : 100.0 N - force maximale appliquée
 * - Epsilon interne : 0.005 m (5 mm) - tolérance minimale
 * - Epsilon externe : 0.010 m (10 mm) - tolérance maximale
 * 
 * Exemples d'objets compatibles :
 * - Marqueur magique : diamètre ~30 mm
 * - Stylo BIC : diamètre ~8 mm (dans la tolérance)
 * - Petite lampe de poche : diamètre ~30 mm
 */
void GripperExampleController::graspGripper() {
  RCLCPP_DEBUG(get_node()->get_logger(), "Closing the gripper - Submitting a Grasp Goal");

  // Définition de l'objectif de prise - adapté pour un "marqueur magique"
  // Largeur anticipée de 30 mm (diamètre du cylindre)
  franka_msgs::action::Grasp::Goal grasp_goal;
  grasp_goal.width = 0.030;        // Largeur cible : 30 mm
  grasp_goal.speed = 0.05;         // Vitesse de fermeture : 0.05 m/s (lente pour précision)
  grasp_goal.force = 100.0;        // Force maximale : 100 N
  grasp_goal.epsilon.inner = 0.005; // Tolérance interne : 5 mm (échec si < 25 mm)
  grasp_goal.epsilon.outer = 0.010; // Tolérance externe : 10 mm (échec si > 40 mm)

  // Envoi asynchrone de l'objectif de prise
  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>>
      grasp_goal_handle =
          gripper_grasp_action_client_->async_send_goal(grasp_goal, grasp_goal_options_);

  // Vérification de la validité de l'envoi
  bool ret = grasp_goal_handle.valid();
  if (ret) {
    RCLCPP_DEBUG(get_node()->get_logger(), "Submitted a Grasp Goal");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), RED "Failed to submit a Grasp Goal" RESET);
  }
}

}  // namespace franka_example_controllers

// Déclaration du plugin pour l'export vers pluginlib
// Permet au système ROS2 de charger dynamiquement ce contrôleur
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::GripperExampleController,
                       controller_interface::ControllerInterface)