#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray

class DistanceParroies(Node):
    """
    Nœud qui calcul la distance entre l'effecteur du robot et les parroies de la cellule.
    """

    def __init__(self):
        super().__init__('distance_parroies')
        
        # Création du subscriber pour la pose de l'effecteur du robot
        self.position_effecteur_robot_sub = self.create_subscription(
            TFMessage, 
            '/position_effecteur_robot', 
            self.position_effecteur_robot_callback, 
            10
        )

        # Création d'un publisher pour la distance
        self.distance_pub = self.create_publisher(
            Float32MultiArray,
            '/distance_robot_parroies',
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
        
        # Timer pour publier les distances à intervalles réguliers
        self.timer = self.create_timer(0.1, self.limite_cellule)  # 10 Hz
        
        self.get_logger().info(f"Nœud de calcul de distance démarré")

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
        
        self.get_logger().debug(f"Position robot reçue: [{self.x}, {self.y}, {self.z}] " +
                                f"[{self.qx}, {self.qy}, {self.qz}, {self.qw}]")

    def limite_cellule(self):
        """
        Permet de calculer la distance entre la position de l'effecteur du robot
        et les bords de la cellule.
        
        Returns:
            list: Liste des distances [dist_x_min, dist_x_max, dist_y_min, dist_y_max, dist_z_min, dist_z_max]
        """       
        # Définir les limites de la cellule
        # Format: [min_x, max_x, min_y, max_y, min_z, max_z]
        limites_cellule = [-1.0, 1.0, -0.5, 0.5, 0.0, 1.0]  # Exemple de limites en mètres
        
        # Calculer les distances aux parois
        distance_x_min = abs(self.x - limites_cellule[0])  # Distance à la paroi X minimum
        distance_x_max = abs(self.x - limites_cellule[1])  # Distance à la paroi X maximum
        distance_y_min = abs(self.y - limites_cellule[2])  # Distance à la paroi Y minimum
        distance_y_max = abs(self.y - limites_cellule[3])  # Distance à la paroi Y maximum
        distance_z_min = abs(self.z - limites_cellule[4])  # Distance à la paroi Z minimum (sol)
        distance_z_max = abs(self.z - limites_cellule[5])  # Distance à la paroi Z maximum (plafond)
        
        # Calculer la distance minimale à chaque axe (distance à la paroi la plus proche)
        distance_x_proche = min(distance_x_min, distance_x_max)
        distance_y_proche = min(distance_y_min, distance_y_max)
        distance_z_proche = min(distance_z_min, distance_z_max)
        
        # Créer le message Float32MultiArray
        msg = Float32MultiArray()
        msg.data = [distance_x_proche, distance_y_proche, distance_z_proche]
        
        # Publier le message
        self.distance_pub.publish(msg)
        
        self.get_logger().debug(f"Distances publiées: suivant x: {distance_x_proche}    suivant y: {distance_y_proche}  suivant z: {distance_z_proche}")
        

def main(args=None):
    rclpy.init(args=args)
    node = DistanceParroies()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()