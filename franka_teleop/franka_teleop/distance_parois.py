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

        # Création d'un publisher pour la force de répulsion
        self.repulsion_pub = self.create_publisher(
            Float32MultiArray,
            '/repulsion_parois',
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

        # Initialiser les distances aux parois
        self.distance_parois_x = 0.0
        self.distance_parois_y = 0.0
        self.distance_parois_z = 0.0

        # Initialiser les forces de répulsions
        self.repulsion_parois_x = 0.0
        self.repulsion_parois_y = 0.0
        self.repulsion_parois_z = 0.0

        # Initialiser les variables du contrôleur et vitesses (nécessaires pour calcul_force)
        self.controleur_x = 0.0
        self.controleur_y = 0.0
        self.controleur_z = 0.0
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0

        # Distance de sécurité
        self.distance_safe_parroies = 0.10  # 10cm de sécurité

        # Compteur pour les logs
        self.count = 0
        
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
        """       
        # Définir les limites de la cellule
        # Format: [min_x, max_x, min_y, max_y, min_z, max_z]
        limites_cellule = [-2.0, 2.0, -1.0, 1.0, -1.0, 2.0]  # Exemple de limites en mètres
        
        # Calculer les distances aux parois
        distance_x_min = abs(self.x - limites_cellule[0])  # Distance à la paroi X minimum
        distance_x_max = abs(self.x - limites_cellule[1])  # Distance à la paroi X maximum
        distance_y_min = abs(self.y - limites_cellule[2])  # Distance à la paroi Y minimum
        distance_y_max = abs(self.y - limites_cellule[3])  # Distance à la paroi Y maximum
        distance_z_min = abs(self.z - limites_cellule[4])  # Distance à la paroi Z minimum (sol)
        distance_z_max = abs(self.z - limites_cellule[5])  # Distance à la paroi Z maximum (plafond)
        
        # Calculer la distance minimale à chaque axe (distance à la paroi la plus proche)
        self.distance_parois_x = min(distance_x_min, distance_x_max)
        self.distance_parois_y = min(distance_y_min, distance_y_max)
        self.distance_parois_z = min(distance_z_min, distance_z_max)
        
        # Créer le message Float32MultiArray pour les distances
        distance_msg = Float32MultiArray()
        distance_msg.data = [self.distance_parois_x, self.distance_parois_y, self.distance_parois_z]
        
        # Publier le message des distances
        self.distance_pub.publish(distance_msg)
        
        # Calculer et publier les forces de répulsion
        self.calcul_force()
        
        self.get_logger().debug(f"Distances publiées: suivant x: {self.distance_parois_x}    suivant y: {self.distance_parois_y}  suivant z: {self.distance_parois_z}")

    def calcul_force(self):
        """
        Permet de calculer la force de répulsion des parois à partir de la distance entre l'effecteur du robot
        et les parroies.
        """
        # Paramètres du ressort
        k_spring = 10.0  # Constante de raideur du ressort (N/m)
        damping = 2.0    # Coefficient d'amortissement (N.s/m)
        
        # Incrémenter le compteur
        self.count += 1
        
        # Calcul des forces de ressort pour chaque axe
        
        # Axe X
        if self.distance_parois_x < self.distance_safe_parroies:
            # Compression du ressort (distance négative depuis la position de repos)
            compression_x = self.distance_safe_parroies - self.distance_parois_x
            
            # Force de rappel proportionnelle à la compression : F = -k * x
            force_rappel_x = k_spring * compression_x
            
            # Force d'amortissement proportionnelle à la vitesse : F = -c * v
            force_amortissement_x = damping * self.speed_x
            
            # Déterminer la direction de la force en fonction de la position du contrôleur
            if self.controleur_x > 0.25:
                self.repulsion_parois_x = -(force_rappel_x + force_amortissement_x)
            else:
                self.repulsion_parois_x = force_rappel_x + force_amortissement_x
                
            # Limiter la force maximale pour éviter les valeurs extrêmes
            self.repulsion_parois_x = max(-20.0, min(20.0, self.repulsion_parois_x))
        else:
            self.repulsion_parois_x = 0.0

        # Axe Y
        if self.distance_parois_y < self.distance_safe_parroies:
            compression_y = self.distance_safe_parroies - self.distance_parois_y
            force_rappel_y = k_spring * compression_y
            force_amortissement_y = damping * self.speed_y
            
            if self.controleur_y > -0.02:
                self.repulsion_parois_y = -(force_rappel_y + force_amortissement_y)
            else:
                self.repulsion_parois_y = force_rappel_y + force_amortissement_y
                
            self.repulsion_parois_y = max(-20.0, min(20.0, self.repulsion_parois_y))
        else:
            self.repulsion_parois_y = 0.0

        # Axe Z
        if self.distance_parois_z < self.distance_safe_parroies:
            compression_z = self.distance_safe_parroies - self.distance_parois_z
            force_rappel_z = k_spring * compression_z
            force_amortissement_z = damping * self.speed_z
            
            if self.controleur_z > 0.05:
                self.repulsion_parois_z = -(force_rappel_z + force_amortissement_z)
            else:
                self.repulsion_parois_z = force_rappel_z + force_amortissement_z
                
            self.repulsion_parois_z = max(-20.0, min(20.0, self.repulsion_parois_z))
        else:
            self.repulsion_parois_z = 0.0

        # Créer le message Float32MultiArray pour les forces de répulsion
        repulsion_msg = Float32MultiArray()
        repulsion_msg.data = [self.repulsion_parois_x, self.repulsion_parois_y, self.repulsion_parois_z]

        # Publier le message des forces de répulsion
        self.repulsion_pub.publish(repulsion_msg)
        
        # Log de debug
        if self.count % 10 == 0:  # Affichage périodique pour éviter le spam
            self.get_logger().debug(f"Forces ressort: x:{self.repulsion_parois_x:.3f} y:{self.repulsion_parois_y:.3f} z:{self.repulsion_parois_z:.3f}")
        

def main(args=None):
    rclpy.init(args=args)
    node = DistanceParroies()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()