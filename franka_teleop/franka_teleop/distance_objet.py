#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import math

class DistanceObjet(Node):
    """
    Nœud qui calcule la distance entre l'effecteur du robot et un objet.
    """

    def __init__(self):
        super().__init__('distance_objet')
        
        # Création du subscriber pour la pose de l'effecteur du robot
        self.position_effecteur_robot_sub = self.create_subscription(
            TFMessage, 
            '/position_effecteur_robot', 
            self.position_effecteur_robot_callback, 
            10
        )

        # Création du subscriber pour la position de l'objet
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/fiole/pose',
            self.pose_callback,
            10
        )

        # Création d'un publisher pour la distance
        self.distance_pub = self.create_publisher(
            Float32MultiArray,
            '/distance_robot_objet',
            10
        )

        # Initialiser la position du robot
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        # Initialiser la position de l'objet
        self.pose_objet_x = 0.0
        self.pose_objet_y = 0.0
        self.pose_objet_z = 0.0
        self.pose_objet_qx = 0.0
        self.pose_objet_qy = 0.0
        self.pose_objet_qz = 0.0
        self.pose_objet_qw = 1.0

        # Flags pour vérifier si les données ont été reçues
        self.robot_data_received = False
        self.object_data_received = False

        # Compteur pour les logs
        self.count = 0
        
        # Timer pour publier les distances à intervalles réguliers
        self.timer = self.create_timer(0.001, self.publish_distance)  # 1000 Hz
        
        self.get_logger().info("Nœud de calcul de distance de l'objet démarré")

    def position_effecteur_robot_callback(self, msg):
        """Callback pour recevoir la position de l'effecteur du robot"""
        if len(msg.transforms) == 0:
            self.get_logger().error("Message de position robot reçu est vide (aucune transformation)")
            return
            
        # Récupérer la première transformation du message
        transform = msg.transforms[0]
        
        # Mettre à jour la position du robot
        self.x = float(transform.transform.translation.x)
        self.y = float(transform.transform.translation.y)
        self.z = float(transform.transform.translation.z)
        self.qx = float(transform.transform.rotation.x)
        self.qy = float(transform.transform.rotation.y)
        self.qz = float(transform.transform.rotation.z)
        self.qw = float(transform.transform.rotation.w)
        
        self.robot_data_received = True
        
        self.get_logger().debug(f"Position robot reçue: [{self.x:.3f}, {self.y:.3f}, {self.z:.3f}] " +
                                f"[{self.qx:.3f}, {self.qy:.3f}, {self.qz:.3f}, {self.qw:.3f}]")

    def pose_callback(self, msg):
        """Callback pour recevoir la position de l'objet"""
        # Extraire la position de l'objet du message
        self.pose_objet_x = float(msg.pose.position.x)
        self.pose_objet_y = float(msg.pose.position.y)
        self.pose_objet_z = float(msg.pose.position.z)
        
        # Correction: utiliser orientation au lieu de rotation
        self.pose_objet_qx = float(msg.pose.orientation.x)
        self.pose_objet_qy = float(msg.pose.orientation.y)
        self.pose_objet_qz = float(msg.pose.orientation.z)
        self.pose_objet_qw = float(msg.pose.orientation.w)
        
        self.object_data_received = True
        
        self.get_logger().debug(f"Position objet reçue: [{self.pose_objet_x:.3f}, {self.pose_objet_y:.3f}, {self.pose_objet_z:.3f}]")

    def calcul_distance(self):
        """Calcule la distance euclidienne entre le robot et l'objet"""
        if not self.robot_data_received or not self.object_data_received:
            self.get_logger().debug("Données manquantes pour le calcul de distance")
            return None, None, None, None
            
        distance = math.sqrt(
            (self.x - self.pose_objet_x)**2 + 
            (self.y - self.pose_objet_y)**2 + 
            (self.z - self.pose_objet_z)**2
        )

        dx = self.x - self.pose_objet_x
        dy = self.y - self.pose_objet_y
        dz = self.z - self.pose_objet_z
        
        return distance, dx, dy, dz

    def publish_distance(self):
        """Publie la distance et ses composantes"""
        result = self.calcul_distance()
        
        if result[0] is None:
            return
            
        distance, dx, dy, dz = result
        
        # Créer le message Float32MultiArray
        # Format: [distance_totale, dx, dy, dz]
        msg = Float32MultiArray()
        msg.data = [float(distance), float(dx), float(dy), float(dz)]
        
        # Publier le message
        self.distance_pub.publish(msg)
        
        # Log périodique (toutes les 50 itérations pour éviter le spam)
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().debug(
                f"Distance: {distance:.3f}m, "
                f"Composantes: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}"
            )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DistanceObjet()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()