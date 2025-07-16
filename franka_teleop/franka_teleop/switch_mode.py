#!/usr/bin/env python3
"""
Module de commutation de contrôleurs pour robot Franka FR3
==========================================================

Ce module implémente un nœud ROS2 qui permet de basculer automatiquement
entre les modes de contrôle position et vitesse d'un robot Franka FR3
lors d'une téléopération avec un contrôleur haptique Desktop 6D de Haption.

Le système fonctionne en simulation sur Gazebo Fortress avec ROS2 Humble.

Auteur: Vincent Bassemayousse
Date: 07/10/2025
Version: 1.0
Licence: Apache 2.0

Dépendances:
    - rclpy (ROS2 Python client library)
    - std_msgs (messages ROS2 standards)
    - controller_manager_msgs (services de gestion des contrôleurs)
    - builtin_interfaces (types de données temporelles)

Usage:
    ros2 run [package_name] switch_mode.py

Topics:
    - Subscriber: /Mode_Pose_Vitesse (Float32MultiArray)
      Format: [mode_position, mode_vitesse] où 1.0 = actif, 0.0 = inactif
    
Services:
    - Client: /controller_manager/switch_controller (SwitchController)
      Utilisé pour basculer entre les contrôleurs ROS2

Contrôleurs gérés:
    - joint_position_example_controller (mode position)
    - joint_velocity_example_controller (mode vitesse)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration
import time

class ControllerSwitcher(Node):
    """
    Nœud ROS2 pour la commutation automatique entre contrôleurs position/vitesse.
    
    Ce nœud écoute les commandes de mode provenant du système de téléopération
    et bascule automatiquement entre les contrôleurs de position et de vitesse
    du robot Franka FR3 selon les besoins.
    
    Attributes:
        in_position_mode (bool): État actuel du mode position
        in_speed_mode (bool): État actuel du mode vitesse
        last_switch_time (float): Timestamp de la dernière commutation
        min_switch_interval (float): Intervalle minimum entre commutations (secondes)
        switch_in_progress (bool): Flag indiquant si une commutation est en cours
        cli (rclpy.Client): Client du service switch_controller
        subscription (rclpy.Subscription): Abonnement au topic de mode
    """

    def __init__(self):
        """
        Initialise le nœud de commutation de contrôleurs.
        
        Configure les états initiaux, crée le client de service et
        l'abonnement au topic de commande de mode.
        """
        super().__init__('controller_switcher')

        # Paramètre pour le namespace
        self.declare_parameter('robot_namespace', '')
        robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        
        # === ÉTATS DES MODES ===
        # Par défaut, le robot démarre en mode position pour la sécurité
        self.in_position_mode = True
        self.in_speed_mode = False
        
        # === PROTECTION CONTRE LES COMMUTATIONS TROP FRÉQUENTES ===
        # Évite les oscillations rapides entre modes qui pourraient
        # déstabiliser le système ou endommager le matériel
        self.last_switch_time = time.time()
        self.min_switch_interval = 1.0  # Intervalle minimum de 1 seconde
        
        # === FLAG DE COMMUTATION EN COURS ===
        # Empêche les commutations simultanées qui pourraient causer
        # des conflits dans le gestionnaire de contrôleurs
        self.switch_in_progress = False

        # Construction du nom du service en fonction du namespace
        if robot_namespace:
            service_name = f'/{robot_namespace}/controller_manager/switch_controller'
        else:
            service_name = '/controller_manager/switch_controller'
            
        self.get_logger().info(f'Tentative de connexion au service: {service_name}')

        # === CLIENT DU SERVICE DE COMMUTATION ===
        # Interface avec le controller_manager de ROS2 pour basculer
        # entre les différents contrôleurs du robot
        self.cli = self.create_client(SwitchController, service_name)

        # Tentative de découverte automatique du service si le service par défaut n'existe pas
        if not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'Service {service_name} non trouvé, tentative de découverte automatique...')
            self.discover_service()

        # === ABONNEMENT AU TOPIC DE COMMANDE DE MODE ===
        # Écoute les commandes provenant du système de téléopération
        # Le topic /Mode_Pose_Vitesse transmet les instructions de mode
        # sous forme de tableau [mode_position, mode_vitesse]
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/Mode_Pose_Vitesse',
            self.mode_position_vitesse_callback,
            10  # Taille de la queue pour éviter la perte de messages
        )
        
        self.get_logger().info('Nœud de commutation de contrôleur démarré')
        self.get_logger().info('Mode initial: POSITION')

    def discover_service(self):
        """Découverte automatique du service switch_controller"""
        import subprocess
        import re
        
        try:
            # Lister tous les services disponibles
            result = subprocess.run(['ros2', 'service', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                services = result.stdout.strip().split('\n')
                switch_services = [s for s in services if 'switch_controller' in s]
                
                if switch_services:
                    service_name = switch_services[0]  # Prendre le premier trouvé
                    self.get_logger().info(f'Service trouvé automatiquement: {service_name}')
                    
                    # Créer un nouveau client avec le bon service
                    self.cli = self.create_client(SwitchController, service_name)
                    
                    if not self.cli.wait_for_service(timeout_sec=5.0):
                        self.get_logger().error(f'Impossible de se connecter au service {service_name}')
                    else:
                        self.get_logger().info(f'Connexion réussie au service {service_name}')
                else:
                    self.get_logger().error('Aucun service switch_controller trouvé')
            else:
                self.get_logger().error('Erreur lors de la découverte des services')
                
        except Exception as e:
            self.get_logger().error(f'Erreur lors de la découverte automatique: {e}')

    def mode_position_vitesse_callback(self, msg):
        """
        Callback pour le traitement des commandes de mode.
        
        Traite les messages reçus sur le topic /Mode_Pose_Vitesse et déclenche
        la commutation appropriée entre les modes position et vitesse.
        
        Args:
            msg (Float32MultiArray): Message contenant les états des modes
                Format attendu: [mode_position, mode_vitesse]
                Valeurs: 1.0 = mode actif, 0.0 = mode inactif
        
        Note:
            - Seul un mode peut être actif à la fois
            - Les commutations trop fréquentes sont filtrées
            - Les commutations simultanées sont évitées
        """
        # Vérifier si le service est disponible
        if not self.cli.service_is_ready():
            self.get_logger().warn('Service switch_controller non disponible')
            return
            
        # Déboguer le message reçu
        #self.get_logger().info(f"Message reçu: {msg.data}")
        
        # === VALIDATION DU FORMAT DU MESSAGE ===
        if len(msg.data) < 2:
            self.get_logger().error(
                f"Message du mode invalide: attendu 2 valeurs, reçu {len(msg.data)}"
            )
            return
            
        # === EXTRACTION DES ÉTATS DE MODE ===
        # Seuil de 0.5 pour éviter les problèmes de bruit ou d'arrondi
        position_mode = msg.data[0] > 0.5
        speed_mode = msg.data[1] > 0.5
        
        # === VALIDATION DE LA COHÉRENCE ===
        # Les deux modes ne peuvent pas être actifs simultanément
        if position_mode and speed_mode:
            self.get_logger().warn(
                "Les deux modes sont actifs dans le message, ceci n'est pas attendu"
            )
            return
            
        # Si aucun mode n'est actif, on maintient l'état actuel
        if not position_mode and not speed_mode:
            self.get_logger().debug("Aucun mode actif dans le message, état maintenu")
            return
        
        # === PROTECTION CONTRE LES COMMUTATIONS TROP FRÉQUENTES ===
        current_time = time.time()
        if self.switch_in_progress:
            self.get_logger().debug("Commutation ignorée: une commutation est déjà en cours")
            return
            
        if (current_time - self.last_switch_time) < self.min_switch_interval:
            self.get_logger().debug(
                f"Commutation ignorée: intervalle minimum non respecté "
                f"({current_time - self.last_switch_time:.2f}s < {self.min_switch_interval}s)"
            )
            return
        

        # === LOGIQUE DE COMMUTATION ===
        # Passage au mode position (depuis le mode vitesse)
        if position_mode and not self.in_position_mode:
            self.switch_controllers(
                'joint_position_example_controller',  # Contrôleur à démarrer
                'cartesian_velocity_example_controller'   # Contrôleur à arrêter
            )
            self.get_logger().info("Demande de passage au mode POSITION")
            
        # Passage au mode vitesse (depuis le mode position)
        elif speed_mode and not self.in_speed_mode:
            self.switch_controllers(
                'cartesian_velocity_example_controller',  # Contrôleur à démarrer
                'joint_position_example_controller'   # Contrôleur à arrêter
            )
            self.get_logger().info("Demande de passage au mode VITESSE")
        
    def switch_controllers(self, controller_to_start, controller_to_stop):
        """
        Effectue la commutation entre contrôleurs via le service ROS2.
        
        Cette méthode prépare et envoie une requête asynchrone au service
        switch_controller du controller_manager pour basculer entre les
        contrôleurs de position et de vitesse.
        
        Args:
            controller_to_stop (str): Nom du contrôleur à arrêter
            controller_to_start (str): Nom du contrôleur à démarrer
        
        Note:
            - La commutation est asynchrone pour éviter de bloquer le nœud
            - Un timeout de 2 secondes est appliqué pour éviter les blocages
            - La stratégie BEST_EFFORT permet une commutation souple
        """
        # Marquer qu'une commutation est en cours
        self.switch_in_progress = True
        
        # Version avec deux appels séparés
        self.controller_to_start = controller_to_start
        self.controller_to_stop = controller_to_stop
        
        # Première étape : désactiver l'ancien contrôleur
        # Stratégie de commutation: BEST_EFFORT permet une commutation
        # souple même si tous les contrôleurs ne peuvent pas être démarrés/arrêtés
        request_deactivate = SwitchController.Request()
        request_deactivate.deactivate_controllers = [controller_to_stop]
        request_deactivate.activate_controllers = []
        request_deactivate.strictness = SwitchController.Request.BEST_EFFORT
        
        # === CONFIGURATION DU TIMEOUT ===
        # Timeout de 2 secondes pour éviter les blocages
        timeout = Duration()
        timeout.sec = 2
        timeout.nanosec = 0
        request_deactivate.timeout = timeout
        
        # === ENVOI DE LA REQUÊTE ASYNCHRONE ===
        future_deactivate = self.cli.call_async(request_deactivate)
        future_deactivate.add_done_callback(self.deactivate_callback)
        
        # Mettre à jour le dernier temps de commutation
        self.last_switch_time = time.time()
    
    def deactivate_callback(self, future):
        """Callback appelé après la désactivation du contrôleur"""
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info(f"Contrôleur {self.controller_to_stop} désactivé avec succès")
                
                # Deuxième étape : activer le nouveau contrôleur
                request_activate = SwitchController.Request()
                request_activate.activate_controllers = [self.controller_to_start]
                request_activate.deactivate_controllers = []
                request_activate.strictness = SwitchController.Request.BEST_EFFORT
                
                # Définir un timeout de 2 secondes
                timeout = Duration()
                timeout.sec = 2
                timeout.nanosec = 0
                request_activate.timeout = timeout
                
                # Envoyer la deuxième requête
                future_activate = self.cli.call_async(request_activate)
                future_activate.add_done_callback(self.activate_callback)
            else:
                self.get_logger().error(f"Échec de la désactivation du contrôleur {self.controller_to_stop}")
                self.switch_in_progress = False
        except Exception as e:
            self.get_logger().error(f"Exception lors de la désactivation: {e}")
            self.switch_in_progress = False
    
    def activate_callback(self, future):
        """Callback appelé après l'activation du contrôleur"""
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info(f"Contrôleur {self.controller_to_start} activé avec succès")
                # Mise à jour des états en fonction de la commutation effectuée
                self.in_position_mode = not self.in_position_mode
                self.in_speed_mode = not self.in_speed_mode
                self.get_logger().info("Commutation des contrôleurs réussie")
            else:
                self.get_logger().error(f"Échec de l'activation du contrôleur {self.controller_to_start}")
        except Exception as e:
            self.get_logger().error(f"Exception lors de l'activation: {e}")
        finally:
            self.switch_in_progress = False
    
def main(args=None):
    """
    Point d'entrée principal du nœud.
    
    Initialise ROS2, crée le nœud ControllerSwitcher et démarre
    la boucle d'événements principale.
    
    Args:
        args (list, optional): Arguments de ligne de commande
    
    Note:
        - Gère proprement l'arrêt du nœud avec destroy_node()
        - Ferme ROS2 avec shutdown() pour libérer les ressources
    """
    # === INITIALISATION DE ROS2 ===
    rclpy.init(args=args)
    
    try:
        # === CRÉATION ET DÉMARRAGE DU NŒUD ===
        node = ControllerSwitcher()
        
        # Boucle d'événements principale (bloquante)
        # Traite les callbacks, services, etc.
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # Gestion propre de l'interruption clavier (Ctrl+C)
        print("\nArrêt demandé par l'utilisateur")
        
    finally:
        # === NETTOYAGE PROPRE ===
        # Destruction du nœud et fermeture de ROS2
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()