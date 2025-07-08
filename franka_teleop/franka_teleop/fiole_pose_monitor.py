#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess
import re
import threading
import time

class FiolePoseMonitor(Node):
    """
    Nœud qui utilise directement la commande 'ign topic' pour récupérer 
    la pose de la fiole et la republie comme un topic ROS 2.
    Si la commande échoue, utilise des valeurs par défaut.
    """

    def __init__(self):
        super().__init__('fiole_pose_monitor')
        
        # Déclarer les paramètres
        self.declare_parameter('model_name', 'Fiole')
        self.declare_parameter('poll_rate', 10.0)  # Hz
        
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.poll_rate = self.get_parameter('poll_rate').get_parameter_value().double_value
        
        # Construire le topic Ignition basé sur le nom du modèle
        self.ignition_topic = f"/model/{self.model_name}/pose"
        
        # Créer le publisher pour la pose de la fiole
        self.publisher = self.create_publisher(
            PoseStamped,
            '/fiole/pose',
            10)
        
        # Valeurs par défaut en cas d'échec de la commande ign topic
        self.default_pose = {
            'position': {'x': 0.45, 'y': 0.0, 'z': 0.42},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        }
        
        # Démarrer le thread de polling
        self.running = True
        self.polling_thread = threading.Thread(target=self.poll_pose)
        self.polling_thread.daemon = True
        self.polling_thread.start()
        
        self.get_logger().info(f"Monitoring de pose démarré pour le modèle: {self.model_name}")
        self.get_logger().info(f"Topic Ignition: {self.ignition_topic}")
        self.get_logger().info("Valeurs par défaut configurées en cas d'échec de la commande ign topic")

    def create_default_pose_msg(self):
        """
        Crée un message PoseStamped avec les valeurs par défaut.
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        
        # Assigner les valeurs par défaut
        pose_msg.pose.position.x = self.default_pose['position']['x']
        pose_msg.pose.position.y = self.default_pose['position']['y']
        pose_msg.pose.position.z = self.default_pose['position']['z']
        
        pose_msg.pose.orientation.x = self.default_pose['orientation']['x']
        pose_msg.pose.orientation.y = self.default_pose['orientation']['y']
        pose_msg.pose.orientation.z = self.default_pose['orientation']['z']
        pose_msg.pose.orientation.w = self.default_pose['orientation']['w']
        
        return pose_msg

    def poll_pose(self):
        """
        Fonction exécutée dans un thread séparé qui interroge périodiquement
        la pose de la fiole via la commande 'ign topic'.
        Utilise des valeurs par défaut en cas d'échec.
        """
        while self.running and rclpy.ok():
            pose_msg = None
            use_default = False
            
            try:
                # Exécuter la commande ign topic en mode 'once'
                cmd = ["ign", "topic", "-e", "-n", "1", "-t", self.ignition_topic]
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
                
                if result.returncode == 0:
                    output = result.stdout
                    
                    # Parser la nouvelle structure de message
                    if "pose {" in output:
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_msg.header.frame_id = "world"
                        
                        # Extraire les valeurs de position
                        x_match = re.search(r"position\s*\{\s*x:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", output)
                        y_match = re.search(r"y:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", output)
                        z_match = re.search(r"z:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", output)
                        
                        # Extraire les valeurs d'orientation
                        orientation_section = re.search(r"orientation\s*\{([^}]+)\}", output)
                        if orientation_section:
                            orientation_text = orientation_section.group(1)
                            qx_match = re.search(r"x:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", orientation_text)
                            qy_match = re.search(r"y:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", orientation_text)
                            qz_match = re.search(r"z:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", orientation_text)
                            qw_match = re.search(r"w:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", orientation_text)
                        else:
                            qx_match = qy_match = qz_match = qw_match = None
                        
                        # Assigner les valeurs au message
                        if x_match and y_match and z_match:
                            pose_msg.pose.position.x = float(x_match.group(1))
                            pose_msg.pose.position.y = float(y_match.group(1))
                            pose_msg.pose.position.z = float(z_match.group(1))
                            
                            pose_msg.pose.orientation.x = float(qx_match.group(1)) if qx_match else 0.0
                            pose_msg.pose.orientation.y = float(qy_match.group(1)) if qy_match else 0.0
                            pose_msg.pose.orientation.z = float(qz_match.group(1)) if qz_match else 0.0
                            pose_msg.pose.orientation.w = float(qw_match.group(1)) if qw_match else 1.0
                            
                            self.get_logger().debug(f"Pose récupérée depuis Ignition pour {self.model_name}: x={pose_msg.pose.position.x:.3f}, y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}")
                        else:
                            self.get_logger().warn("Impossible d'extraire les coordonnées de position, utilisation des valeurs par défaut")
                            use_default = True
                    else:
                        self.get_logger().warn("Format de message pose non reconnu, utilisation des valeurs par défaut")
                        use_default = True
                else:
                    self.get_logger().warn(f"Erreur commande ign topic: {result.stderr}, utilisation des valeurs par défaut")
                    use_default = True
                
            except subprocess.TimeoutExpired:
                self.get_logger().debug("Délai d'attente dépassé pour la commande 'ign topic', utilisation des valeurs par défaut")
                use_default = True
            except Exception as e:
                self.get_logger().warn(f"Erreur lors de la récupération de la pose: {str(e)}, utilisation des valeurs par défaut")
                use_default = True
            
            # Si on n'a pas réussi à récupérer la pose ou si elle est invalide, utiliser les valeurs par défaut
            if use_default or pose_msg is None:
                pose_msg = self.create_default_pose_msg()
                self.get_logger().debug(f"Pose par défaut utilisée pour {self.model_name}: x={pose_msg.pose.position.x:.3f}, y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}")
            
            # Publier le message (qu'il vienne d'Ignition ou des valeurs par défaut)
            self.publisher.publish(pose_msg)
            
            # Attendre avant la prochaine interrogation
            time.sleep(5.0 / self.poll_rate)

    def destroy_node(self):
        self.running = True
        if self.polling_thread.is_alive():
            self.polling_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    fiole_pose_monitor = FiolePoseMonitor()
    
    try:
        rclpy.spin(fiole_pose_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        fiole_pose_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()