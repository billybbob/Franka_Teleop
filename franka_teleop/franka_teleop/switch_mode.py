#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from controller_manager_msgs.srv import SwitchController
from builtin_interfaces.msg import Duration
import time

class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__('controller_switcher')

        # Paramètre pour le namespace (optionnel)
        self.declare_parameter('robot_namespace', '')
        robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        
        # État des modes
        self.in_position_mode = True  # Par défaut, on est en mode position
        self.in_speed_mode = False
        
        # Pour éviter les commutations trop fréquentes
        self.last_switch_time = time.time()
        self.min_switch_interval = 1.0
        
        # Flag pour suivre l'état de la commutation en cours
        self.switch_in_progress = False

        # Construction du nom du service en fonction du namespace
        if robot_namespace:
            service_name = f'/{robot_namespace}/controller_manager/switch_controller'
        else:
            service_name = '/controller_manager/switch_controller'
            
        self.get_logger().info(f'Tentative de connexion au service: {service_name}')

        # Client du service switch_controller
        self.cli = self.create_client(SwitchController, service_name)

        # Tentative de découverte automatique du service si le service par défaut n'existe pas
        if not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'Service {service_name} non trouvé, tentative de découverte automatique...')
            self.discover_service()

        # Subscriber au topic /Mode_Pose_Vitesse
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/Mode_Pose_Vitesse',
            self.mode_position_vitesse_callback,
            10
        )
        
        self.get_logger().info('Nœud de commutation de contrôleur démarré')

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
        """Callback pour le topic Mode_Pose_Vitesse"""
        # Vérifier si le service est disponible
        if not self.cli.service_is_ready():
            self.get_logger().warn('Service switch_controller non disponible')
            return
            
        # Déboguer le message reçu
        #self.get_logger().info(f"Message reçu: {msg.data}")
        
        if len(msg.data) < 2:
            self.get_logger().error(f"Message du mode invalide: attendu 2 valeurs, reçu {len(msg.data)}")
            return
            
        # Dans le message: [0] = mode position (1.0 = actif), [1] = mode vitesse (1.0 = actif)
        position_mode = msg.data[0] > 0.5
        speed_mode = msg.data[1] > 0.5
        
        # Vérifier la cohérence des données
        if position_mode and speed_mode:
            self.get_logger().warn("Les deux modes sont actifs dans le message, ceci n'est pas attendu")
            return  # Ne pas procéder si les deux modes sont actifs
            
        # Si aucun mode n'est actif, ignorer
        if not position_mode and not speed_mode:
            self.get_logger().warn("Aucun mode n'est actif dans le message, ignoré")
            return
        
        # Vérifier si une commutation est déjà en cours ou si l'intervalle minimum n'est pas respecté
        current_time = time.time()
        if self.switch_in_progress or (current_time - self.last_switch_time) < self.min_switch_interval:
            self.get_logger().info("Commutation ignorée: trop fréquente ou déjà en cours")
            return
        
        # Passage au mode position
        if position_mode and not self.in_position_mode:
            self.switch_controllers('joint_position_example_controller', 'cartesian_velocity_example_controller')
            self.get_logger().info("Passage au mode POSITION")
            
        # Passage au mode vitesse
        elif speed_mode and not self.in_speed_mode:
            self.switch_controllers('cartesian_velocity_example_controller', 'joint_position_example_controller')
            self.get_logger().info("Passage au mode VITESSE")
    
    def switch_controllers(self, controller_to_start, controller_to_stop):
        """Fonction pour demander la commutation des contrôleurs via le service"""
        self.switch_in_progress = True
        
        # Version avec deux appels séparés si nécessaire
        self.controller_to_start = controller_to_start
        self.controller_to_stop = controller_to_stop
        
        # Première étape : désactiver l'ancien contrôleur
        request_deactivate = SwitchController.Request()
        request_deactivate.deactivate_controllers = [controller_to_stop]
        request_deactivate.activate_controllers = []
        request_deactivate.strictness = SwitchController.Request.BEST_EFFORT
        
        # Définir un timeout de 2 secondes
        timeout = Duration()
        timeout.sec = 2
        timeout.nanosec = 0
        request_deactivate.timeout = timeout
        
        # Envoyer la première requête
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
    rclpy.init(args=args)
    node = ControllerSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()