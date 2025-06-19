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
        
        # Démarrer le thread de polling
        self.running = True
        self.polling_thread = threading.Thread(target=self.poll_pose)
        self.polling_thread.daemon = True
        self.polling_thread.start()
        
        self.get_logger().info(f"Monitoring de pose démarré pour le modèle: {self.model_name}")
        self.get_logger().info(f"Topic Ignition: {self.ignition_topic}")

    def poll_pose(self):
        """
        Fonction exécutée dans un thread séparé qui interroge périodiquement
        la pose de la fiole via la commande 'ign topic'.
        """
        while self.running and rclpy.ok():
            try:
                # Exécuter la commande ign topic en mode 'once'
                cmd = ["ign", "topic", "-e", "-n", "1", "-t", self.ignition_topic]
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=1.0)
                
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
                            
                            # Publier le message
                            self.publisher.publish(pose_msg)
                            self.get_logger().debug(f"Pose publiée pour {self.model_name}: x={pose_msg.pose.position.x:.3f}, y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}")
                        else:
                            self.get_logger().warn("Impossible d'extraire les coordonnées de position")
                    else:
                        self.get_logger().warn("Format de message pose non reconnu")
                else:
                    self.get_logger().error(f"Erreur commande ign topic: {result.stderr}")
                
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
