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

        # État des modes
        self.in_position_mode = True  # Par défaut, on est en mode position
        self.in_speed_mode = False
        
        # Pour éviter les commutations trop fréquentes
        self.last_switch_time = time.time()
        self.min_switch_interval = 1.0
        
        # Flag pour suivre l'état de la commutation en cours
        self.switch_in_progress = False

        # Client du service switch_controller
        self.cli = self.create_client(SwitchController, '/controller_manager/switch_controller')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attente du service switch_controller...')

        # Subscriber au topic /Mode_Pose_Vitesse
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/Mode_Pose_Vitesse',
            self.mode_position_vitesse_callback,
            10
        )
        
        self.get_logger().info('Nœud de commutation de contrôleur démarré')

    def mode_position_vitesse_callback(self, msg):
        """Callback pour le topic Mode_Pose_Vitesse"""
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
            self.switch_controllers('joint_velocity_example_controller', 'joint_position_example_controller')
            self.get_logger().info("Passage au mode POSITION")
            
        # Passage au mode vitesse
        elif speed_mode and not self.in_speed_mode:
            self.switch_controllers('joint_position_example_controller', 'joint_velocity_example_controller')
            self.get_logger().info("Passage au mode VITESSE")
    
    def switch_controllers(self, controller_to_stop, controller_to_start):
        """Fonction pour demander la commutation des contrôleurs via le service"""
        self.switch_in_progress = True
        
        # Préparer la requête
        request = SwitchController.Request()
        request.start_controllers = [controller_to_start]
        request.stop_controllers = [controller_to_stop]
        request.strictness = SwitchController.Request.BEST_EFFORT
        
        # Définir un timeout de 2 secondes
        timeout = Duration()
        timeout.sec = 2
        timeout.nanosec = 0
        request.timeout = timeout
        
        # Envoyer la requête
        future = self.cli.call_async(request)
        future.add_done_callback(self.switch_callback)
        
        # Mettre à jour le dernier temps de commutation
        self.last_switch_time = time.time()
    
    def switch_callback(self, future):
        """Callback appelé quand la requête de commutation est terminée"""
        try:
            response = future.result()
            if response.ok:
                # Mise à jour des états en fonction de la commutation effectuée
                self.in_position_mode = not self.in_position_mode
                self.in_speed_mode = not self.in_speed_mode
                self.get_logger().info("Commutation des contrôleurs réussie")
            else:
                self.get_logger().error("Échec de la commutation des contrôleurs")
        except Exception as e:
            self.get_logger().error(f"Exception lors de la commutation: {e}")
        finally:
            self.switch_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = ControllerSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main