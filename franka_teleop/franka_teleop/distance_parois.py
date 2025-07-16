#!/usr/bin/env python3
"""
Nœud ROS2 pour le calcul de distances entre l'effecteur du robot et les parois de la cellule,
ainsi que le calcul des forces de répulsion associées.

Ce module est utilisé dans le cadre d'un système de téléopération haptique avec un
contrôleur Desktop 6D (Haption) pour le robot Franka FR3. Il permet de simuler en
temps réel un environnement contraint en générant des forces de répulsion lorsque
l'effecteur du robot s'approche des limites de la cellule virtuelle.

Auteur: Vincent Bassemayousse
Date: 07/10/2025
Version: 1.0
Licence: Apache 2.0

Dépendances :
    - rclpy : Client ROS2 Python
    - tf2_msgs : Messages de transformations
    - std_msgs : Messages standards
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray


class DistanceParroies(Node):
    """
    Nœud ROS2 de calcul de distances entre l'effecteur du robot et les parois virtuelles
    d'une cellule de travail. Génère également des forces de répulsion si le robot
    s'approche trop des limites définies.

    Fonctionnalités :
    - Récupération de la position de l'effecteur (TF)
    - Calcul des distances à chaque paroi (X, Y, Z)
    - Calcul des forces de répulsion (modèle masse-ressort-amortisseur)
    - Publication des distances et forces de répulsion

    Topics souscrits :
        /position_effecteur_robot (tf2_msgs/TFMessage) : Pose actuelle de l'effecteur

    Topics publiés :
        /distance_robot_parroies (std_msgs/Float32MultiArray) : Distances min [dx, dy, dz]
        /repulsion_parois (std_msgs/Float32MultiArray) : Forces de répulsion [Fx, Fy, Fz]
    """

    def __init__(self):
        """
        Initialise le nœud ROS2, les abonnements, les publications, les variables d'état
        ainsi qu'un timer pour le calcul périodique.
        """
        super().__init__('distance_parroies')
        
        # === Souscriptions ===
        self.position_effecteur_robot_sub = self.create_subscription(
            TFMessage,
            '/position_effecteur_robot',
            self.position_effecteur_robot_callback,
            10
        )

        # === Publications ===
        self.distance_pub = self.create_publisher(
            Float32MultiArray,
            '/distance_robot_parroies',
            10
        )
        
        self.repulsion_pub = self.create_publisher(
            Float32MultiArray,
            '/repulsion_parois',
            10
        )

        # === État interne de la position de l'effecteur ===
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.qx, self.qy, self.qz, self.qw = 0.0, 0.0, 0.0, 1.0

        # === Distances aux parois ===
        self.distance_parois_x = 0.0
        self.distance_parois_y = 0.0
        self.distance_parois_z = 0.0

        # === Forces de répulsion ===
        self.repulsion_parois_x = 0.0
        self.repulsion_parois_y = 0.0
        self.repulsion_parois_z = 0.0

        # === État du contrôleur haptique ===
        self.controleur_x = 0.0
        self.controleur_y = 0.0
        self.controleur_z = 0.0

        # === Vitesses estimées (utilisées dans l'amortissement) ===
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0

        # === Distance minimale autorisée à une paroi ===
        self.distance_safe_parroies = 0.10  # 10 cm de marge

        # === Timer pour le traitement périodique (10 Hz) ===
        self.timer = self.create_timer(0.1, self.limite_cellule)

        # Compteur pour debug
        self.count = 0

        self.get_logger().info("Nœud de calcul de distance aux parois initialisé.")

    def position_effecteur_robot_callback(self, msg: TFMessage):
        """
        Callback ROS2 appelé à la réception de la position de l'effecteur du robot.

        Args:
            msg (TFMessage): Message contenant une ou plusieurs transformations (TF)
        """
        if not msg.transforms:
            self.get_logger().error("TFMessage reçu sans données.")
            return

        # On récupère uniquement la première transformation pour la position
        transform = msg.transforms[0]

        self.x = float(transform.transform.translation.x)
        self.y = float(transform.transform.translation.y)
        self.z = float(transform.transform.translation.z)
        self.qx = float(transform.transform.rotation.x)
        self.qy = float(transform.transform.rotation.y)
        self.qz = float(transform.transform.rotation.z)
        self.qw = float(transform.transform.rotation.w)

        self.get_logger().debug(
            f"Pose reçue: Position=({self.x:.3f}, {self.y:.3f}, {self.z:.3f}) | "
            f"Orientation=({self.qx:.3f}, {self.qy:.3f}, {self.qz:.3f}, {self.qw:.3f})"
        )

    def limite_cellule(self):
        """
        Calcul des distances entre l'effecteur et les limites de la cellule, puis publication
        des distances et calcul des forces de répulsion si nécessaire.
        """
        # Limites de la cellule (en mètres)
        # Format: [min_x, max_x, min_y, max_y, min_z, max_z]
        limites_cellule = [-2.0, 2.0, 2.0, 2.0, -2.0, 2.0]  # Exemple de limites en mètres

        # Distances aux bords pour chaque axe
        distance_x_min = abs(self.x - limites_cellule[0])  # Distance à la paroi X minimum
        distance_x_max = abs(self.x - limites_cellule[1])  # Distance à la paroi X maximum
        distance_y_min = abs(self.y - limites_cellule[2])  # Distance à la paroi Y minimum
        distance_y_max = abs(self.y - limites_cellule[3])  # Distance à la paroi Y maximum
        distance_z_min = abs(self.z - limites_cellule[4])  # Distance à la paroi Z minimum (sol)
        distance_z_max = abs(self.z - limites_cellule[5])  # Distance à la paroi Z maximum (plafond)

        # Distance minimale sur chaque axe
        self.distance_parois_x = min(distance_x_min, distance_x_max)
        self.distance_parois_y = min(distance_y_min, distance_y_max)
        self.distance_parois_z = min(distance_z_min, distance_z_max)

        # Publication des distances
        distance_msg = Float32MultiArray()
        distance_msg.data = [self.distance_parois_x, self.distance_parois_y, self.distance_parois_z]
        
        # Publier le message des distances
        self.distance_pub.publish(distance_msg)

        # Calcul des forces de répulsion si nécessaire
        self.calcul_force()

        self.get_logger().debug(
            f"Distances: X={self.distance_parois_x:.3f}m, Y={self.distance_parois_y:.3f}m, Z={self.distance_parois_z:.3f}m"
        )

    def calcul_force(self):
        """
        Calcule les forces de répulsion selon un modèle masse-ressort-amortisseur.
        Applique une force proportionnelle à la compression et à la vitesse si la
        distance minimale est inférieure à la distance de sécurité.
        """
       # Paramètres du ressort
        k_spring = 10.0  # Constante de raideur du ressort (N/m)
        damping = 2.0    # Coefficient d'amortissement (N.s/m)

        # Incrémenter le compteur
        self.count += 1

        # === Axe X ===
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


        # === Axe Y ===
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

        # === Axe Z ===
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

        # Publication des forces
        repulsion_msg = Float32MultiArray()
        repulsion_msg.data = [
            self.repulsion_parois_x,
            self.repulsion_parois_y,
            self.repulsion_parois_z
        ]

        # Publier le message des forces de répulsion
        self.repulsion_pub.publish(repulsion_msg)

         # Log de debug
        if self.count % 10 == 0:  # Affichage périodique pour éviter le spam
            self.get_logger().debug(f"Forces ressort: x:{self.repulsion_parois_x:.3f} y:{self.repulsion_parois_y:.3f} z:{self.repulsion_parois_z:.3f}")


def main(args=None):
    """
    Fonction principale ROS2 pour l'exécution du nœud.
    Initialise, lance et ferme proprement le nœud DistanceParroies.
    """
    rclpy.init(args=args)
    node = DistanceParroies()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

