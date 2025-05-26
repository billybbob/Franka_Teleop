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
    """

    def __init__(self):
        super().__init__('fiole_pose_monitor')
        
        # Déclarer les paramètres
        self.declare_parameter('object_name', 'Fiole_world')
        self.declare_parameter('poll_rate', 10.0)  # Hz
        
        self.object_name = self.get_parameter('object_name').get_parameter_value().string_value
        self.poll_rate = self.get_parameter('poll_rate').get_parameter_value().double_value
        
        # Créer le publisher pour la pose de la fiole
        self.publisher = self.create_publisher(
            PoseStamped,
            '/fiole/pose',
            10)
        
        # Démarrer le thread de polling
        self.running = True
        self.polling_thread = threading.Thread(target=self.poll_pose)
        self.polling_thread.daemon = True
        self.polling_thread.start()
        
        self.get_logger().info(f"Monitoring de pose démarré pour l'objet: {self.object_name}")

    def poll_pose(self):
        """
        Fonction exécutée dans un thread séparé qui interroge périodiquement
        la pose de la fiole via la commande 'ign topic'.
        """
        while self.running and rclpy.ok():
            try:
                # Exécuter la commande ign topic en mode 'once'
                cmd = ["ign", "topic", "-e", "-n", "1", "-t", "/world/demo/pose/info"]
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=1.0)
                
                if result.returncode == 0:
                    output = result.stdout
                    
                    # Chercher notre objet dans la sortie
                    if self.object_name in output:
                        # Extraire la pose avec des expressions régulières
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_msg.header.frame_id = "world"
                        
                        # Trouver la section de notre objet
                        object_pattern = rf"name: \"{self.object_name}\".*?position.*?{{(.*?)}}.*?orientation.*?{{(.*?)}}"
                        object_match = re.search(object_pattern, output, re.DOTALL)
                        
                        if object_match:
                            position_text = object_match.group(1)
                            orientation_text = object_match.group(2)
                            
                            # Extraire x, y, z
                            x_match = re.search(r"x: ([-+]?\d*\.?\d+)", position_text)
                            y_match = re.search(r"y: ([-+]?\d*\.?\d+)", position_text)
                            z_match = re.search(r"z: ([-+]?\d*\.?\d+)", position_text)
                            
                            # Extraire orientation
                            qx_match = re.search(r"x: ([-+]?\d*\.?\d+)", orientation_text)
                            qy_match = re.search(r"y: ([-+]?\d*\.?\d+)", orientation_text)
                            qz_match = re.search(r"z: ([-+]?\d*\.?\d+)", orientation_text)
                            qw_match = re.search(r"w: ([-+]?\d*\.?\d+)", orientation_text)
                            
                            # Assigner les valeurs au message
                            pose_msg.pose.position.x = float(x_match.group(1)) if x_match else 0.0
                            pose_msg.pose.position.y = float(y_match.group(1)) if y_match else 0.0
                            pose_msg.pose.position.z = float(z_match.group(1)) if z_match else 0.0
                            
                            pose_msg.pose.orientation.x = float(qx_match.group(1)) if qx_match else 0.0
                            pose_msg.pose.orientation.y = float(qy_match.group(1)) if qy_match else 0.0
                            pose_msg.pose.orientation.z = float(qz_match.group(1)) if qz_match else 0.0
                            pose_msg.pose.orientation.w = float(qw_match.group(1)) if qw_match else 1.0
                            
                            # Publier le message
                            self.publisher.publish(pose_msg)
                            self.get_logger().debug(f"Pose publiée pour {self.object_name}: x={pose_msg.pose.position.x}, y={pose_msg.pose.position.y}, z={pose_msg.pose.position.z}")
                
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Délai d'attente dépassé pour la commande 'ign topic'")
            except Exception as e:
                self.get_logger().error(f"Erreur lors de la récupération de la pose: {str(e)}")
            
            # Attendre avant la prochaine interrogation
            time.sleep(1.0 / self.poll_rate)

    def destroy_node(self):
        self.running = False
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