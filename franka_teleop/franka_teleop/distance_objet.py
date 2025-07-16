#!/usr/bin/env python3
"""
Nœud ROS2 pour le calcul de distance entre l'effecteur du robot et un objet cible.

Ce module fait partie du système de téléopération haptique pour le robot Franka FR3.
Il calcule en temps réel la distance euclidienne entre l'effecteur du robot et un objet
détecté, ainsi que les composantes directionnelles de cette distance.

Auteur: Vincent Bassemayousse
Date: 07/10/2025
Version: 1.0
Licence: Apache 2.0

Dépendances:
    - rclpy: Framework ROS2 Python
    - tf2_msgs: Messages de transformation
    - std_msgs: Messages standards
    - geometry_msgs: Messages géométriques
    - math: Calculs mathématiques
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import math
from typing import Optional, Tuple


class DistanceObjet(Node):
    """
    Nœud ROS2 pour calculer la distance entre l'effecteur du robot et un objet cible.
    
    Ce nœud s'intègre dans le système de téléopération haptique en fournissant des
    informations de distance utilisables pour la génération de forces de retour
    haptique ou pour la guidance de l'opérateur.
    
    Fonctionnalités principales:
    - Calcul de la distance euclidienne en temps réel
    - Décomposition vectorielle (dx, dy, dz)
    - Publication haute fréquence (1000Hz)
    - Validation des données d'entrée
    - Logging détaillé pour le débogage
    
    Topics souscrits:
        /position_effecteur_robot (tf2_msgs/TFMessage): Position de l'effecteur
        /fiole/pose (geometry_msgs/PoseStamped): Position de l'objet cible
        
    Topics publiés:
        /distance_robot_objet (std_msgs/Float32MultiArray): [distance, dx, dy, dz]
    """

    def __init__(self):
        """
        Initialise le nœud de calcul de distance.
        
        Configure les souscriptions, publications et variables d'état nécessaires
        au fonctionnement du système de mesure de distance.
        """
        super().__init__('distance_objet')
        
        # === Configuration des Topics ROS2 ===
        
        # Souscription pour la position de l'effecteur du robot
        self.position_effecteur_robot_sub = self.create_subscription(
            TFMessage, 
            '/position_effecteur_robot', 
            self.position_effecteur_robot_callback, 
            10  # Taille de la queue
        )

        # Souscription pour la position de l'objet cible
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/fiole/pose',  # Topic spécifique à l'objet "fiole"
            self.pose_callback,
            10  # Taille de la queue
        )

        # Publication pour les données de distance
        self.distance_pub = self.create_publisher(
            Float32MultiArray,
            '/distance_robot_objet',
            10  # Taille de la queue
        )

        # === Variables d'État du Robot ===
        
        # Position cartésienne de l'effecteur (en mètres)
        self.x: float = 0.0      # Position X dans le repère monde
        self.y: float = 0.0      # Position Y dans le repère monde  
        self.z: float = 0.0      # Position Z dans le repère monde
        
        # Orientation de l'effecteur (quaternion normalisé)
        self.qx: float = 0.0     # Composante X du quaternion
        self.qy: float = 0.0     # Composante Y du quaternion
        self.qz: float = 0.0     # Composante Z du quaternion
        self.qw: float = 1.0     # Composante W du quaternion (partie réelle)

        # === Variables d'État de l'Objet ===
        
        # Position cartésienne de l'objet cible (en mètres)
        self.pose_objet_x: float = 0.0    # Position X de l'objet
        self.pose_objet_y: float = 0.0    # Position Y de l'objet
        self.pose_objet_z: float = 0.0    # Position Z de l'objet
        
        # Orientation de l'objet (quaternion normalisé)
        self.pose_objet_qx: float = 0.0   # Composante X du quaternion objet
        self.pose_objet_qy: float = 0.0   # Composante Y du quaternion objet
        self.pose_objet_qz: float = 0.0   # Composante Z du quaternion objet
        self.pose_objet_qw: float = 1.0   # Composante W du quaternion objet

        # === Flags de Validation des Données ===
        
        # Indique si les données du robot ont été reçues au moins une fois
        self.robot_data_received: bool = False
        
        # Indique si les données de l'objet ont été reçues au moins une fois
        self.object_data_received: bool = False

        # === Variables de Contrôle ===
        
        # Compteur pour la gestion des logs périodiques
        self.count: int = 0
        
        # === Configuration du Timer ===
        
        # Timer pour la publication périodique à haute fréquence
        # Fréquence: 1000Hz (1ms) pour une réactivité maximale
        self.timer = self.create_timer(0.001, self.publish_distance)
        
        # === Messages de Démarrage ===
        
        self.get_logger().info("=== Nœud de calcul de distance démarré ===")
        self.get_logger().info("Topics souscrits:")
        self.get_logger().info("  - /position_effecteur_robot (Position robot)")
        self.get_logger().info("  - /fiole/pose (Position objet)")
        self.get_logger().info("Topics publiés:")
        self.get_logger().info("  - /distance_robot_objet (Distance calculée)")
        self.get_logger().info("Fréquence de publication: 1000Hz")

    def position_effecteur_robot_callback(self, msg: TFMessage) -> None:
        """
        Callback pour traiter les messages de position de l'effecteur du robot.
        
        Cette méthode est appelée à chaque réception d'un message TFMessage
        contenant la transformation courante de l'effecteur du robot.
        
        Args:
            msg (TFMessage): Message contenant les transformations TF2.
                            Structure attendue: transforms[0] = transformation effecteur
                            
        Note:
            Le message TFMessage peut contenir plusieurs transformations.
            Cette implémentation utilise la première transformation (index 0)
            qui correspond conventionnellement à l'effecteur du robot.
            
        Raises:
            Logs d'erreur si le message est vide ou mal formé.
        """
        # Validation de la structure du message
        if len(msg.transforms) == 0:
            self.get_logger().error(
                "Message de position robot reçu est vide (aucune transformation)"
            )
            return
            
        # Extraction de la première transformation (effecteur du robot)
        transform = msg.transforms[0]
        
        # === Mise à jour de la Position Cartésienne ===
        
        # Extraction des coordonnées de translation
        self.x = float(transform.transform.translation.x)
        self.y = float(transform.transform.translation.y)
        self.z = float(transform.transform.translation.z)
        
        # === Mise à jour de l'Orientation ===
        
        # Extraction des composantes du quaternion d'orientation
        self.qx = float(transform.transform.rotation.x)
        self.qy = float(transform.transform.rotation.y)
        self.qz = float(transform.transform.rotation.z)
        self.qw = float(transform.transform.rotation.w)
        
        # === Mise à jour du Flag de Validation ===
        
        # Marquer que les données robot sont disponibles
        self.robot_data_received = True
        
        # === Logging de Débogage ===
        
        self.get_logger().debug(
            f"Position robot mise à jour: "
            f"Pos=[{self.x:.3f}, {self.y:.3f}, {self.z:.3f}] "
            f"Rot=[{self.qx:.3f}, {self.qy:.3f}, {self.qz:.3f}, {self.qw:.3f}]"
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        """
        Callback pour traiter les messages de position de l'objet cible.
        
        Cette méthode est appelée à chaque réception d'un message PoseStamped
        contenant la pose courante de l'objet à suivre.
        
        Args:
            msg (PoseStamped): Message contenant la pose de l'objet.
                              Structure: pose.position (x,y,z) + pose.orientation (qx,qy,qz,qw)
                              
        Note:
            Le message PoseStamped inclut un header avec timestamp et frame_id
            qui pourraient être utilisés pour des vérifications de cohérence
            temporelle dans des versions futures.
        """
        # === Extraction de la Position de l'Objet ===
        
        # Mise à jour des coordonnées cartésiennes de l'objet
        self.pose_objet_x = float(msg.pose.position.x)
        self.pose_objet_y = float(msg.pose.position.y)
        self.pose_objet_z = float(msg.pose.position.z)
        
        # === Extraction de l'Orientation de l'Objet ===
        
        # Mise à jour du quaternion d'orientation de l'objet
        # Note: Correction par rapport à la version précédente qui utilisait
        # incorrectement 'rotation' au lieu de 'orientation'
        self.pose_objet_qx = float(msg.pose.orientation.x)
        self.pose_objet_qy = float(msg.pose.orientation.y)
        self.pose_objet_qz = float(msg.pose.orientation.z)
        self.pose_objet_qw = float(msg.pose.orientation.w)
        
        # === Mise à jour du Flag de Validation ===
        
        # Marquer que les données objet sont disponibles
        self.object_data_received = True
        
        # === Logging de Débogage ===
        
        self.get_logger().debug(
            f"Position objet mise à jour: "
            f"[{self.pose_objet_x:.3f}, {self.pose_objet_y:.3f}, {self.pose_objet_z:.3f}]"
        )

    def calcul_distance(self) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
        """
        Calcule la distance euclidienne et ses composantes entre robot et objet.
        
        Cette méthode implémente le calcul de distance 3D en utilisant la formule
        euclidienne standard: d = sqrt((x2-x1)² + (y2-y1)² + (z2-z1)²)
        
        Returns:
            Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
                - distance: Distance euclidienne totale (en mètres)
                - dx: Composante X de la différence de position (robot_x - objet_x)
                - dy: Composante Y de la différence de position (robot_y - objet_y)  
                - dz: Composante Z de la différence de position (robot_z - objet_z)
                
                Retourne (None, None, None, None) si les données ne sont pas disponibles.
                
        Note:
            Les composantes dx, dy, dz représentent le vecteur qui va de l'objet
            vers le robot. Des valeurs positives indiquent que le robot est du côté
            positif de l'axe par rapport à l'objet.
            
        Exemple:
            Si robot=(1,2,3) et objet=(0,1,2), alors:
            - distance = sqrt(1² + 1² + 1²) = 1.732
            - dx = 1, dy = 1, dz = 1
        """
        # === Validation des Données d'Entrée ===
        
        if not self.robot_data_received or not self.object_data_received:
            self.get_logger().debug(
                f"Données manquantes pour le calcul de distance: "
                f"Robot={self.robot_data_received}, Objet={self.object_data_received}"
            )
            return None, None, None, None
        
        # === Calcul des Composantes Directionnelles ===
        
        # Calcul des différences de position sur chaque axe
        # Convention: robot - objet (vecteur pointant vers le robot)
        dx = self.x - self.pose_objet_x
        dy = self.y - self.pose_objet_y
        dz = self.z - self.pose_objet_z
        
        # === Calcul de la Distance Euclidienne ===
        
        # Application de la formule de distance 3D
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # === Logging de Débogage Détaillé ===
        
        self.get_logger().debug(
            f"Calcul distance: "
            f"Robot=[{self.x:.3f},{self.y:.3f},{self.z:.3f}] "
            f"Objet=[{self.pose_objet_x:.3f},{self.pose_objet_y:.3f},{self.pose_objet_z:.3f}] "
            f"=> Distance={distance:.3f}m"
        )
        
        return distance, dx, dy, dz

    def publish_distance(self) -> None:
        """
        Publie les données de distance calculées sur le topic ROS2.
        
        Cette méthode est appelée périodiquement par le timer (1000Hz) pour
        publier les informations de distance sous forme de message Float32MultiArray.
        
        Format du message publié:
            data[0]: Distance euclidienne totale (float, en mètres)
            data[1]: Composante dx (float, en mètres)
            data[2]: Composante dy (float, en mètres)
            data[3]: Composante dz (float, en mètres)
            
        Note:
            La publication n'a lieu que si les données de distance sont valides.
            Un système de logging périodique évite le spam dans les logs.
        """
        # === Calcul des Données de Distance ===
        
        result = self.calcul_distance()
        
        # Vérification de la validité des données
        if result[0] is None:
            # Pas de données valides disponibles, pas de publication
            return
            
        # Extraction des résultats
        distance, dx, dy, dz = result
        
        # === Création du Message ROS2 ===
        
        # Initialisation du message Float32MultiArray
        msg = Float32MultiArray()
        
        # Remplissage des données dans l'ordre défini
        # Format: [distance_totale, composante_x, composante_y, composante_z]
        msg.data = [
            float(distance),  # Distance euclidienne totale
            float(dx),        # Composante X (robot_x - objet_x)
            float(dy),        # Composante Y (robot_y - objet_y)
            float(dz)         # Composante Z (robot_z - objet_z)
        ]
        
        # === Publication du Message ===
        
        self.distance_pub.publish(msg)
        
        # === Logging Périodique ===
        
        # Gestion du compteur pour éviter le spam dans les logs
        self.count += 1
        
        # Log toutes les 50 publications (soit environ 20 fois par seconde à 1000Hz)
        if self.count % 50 == 0:
            self.get_logger().debug(
                f"Distance publiée: {distance:.3f}m, "
                f"Composantes: dx={dx:.3f}m, dy={dy:.3f}m, dz={dz:.3f}m"
            )
            
            # Log d'information périodique moins fréquent
            if self.count % 1000 == 0:  # Toutes les secondes
                self.get_logger().info(
                    f"Système actif - Distance courante: {distance:.3f}m"
                )


def main(args=None):
    """
    Point d'entrée principal du nœud de calcul de distance.
    
    Cette fonction initialise ROS2, crée une instance du nœud DistanceObjet,
    et gère le cycle de vie du nœud avec une gestion appropriée des exceptions.
    
    Args:
        args: Arguments de ligne de commande passés à ROS2 (optionnel)
        
    Note:
        La fonction gère proprement l'arrêt du nœud lors d'une interruption
        clavier (Ctrl+C) et s'assure que toutes les ressources sont libérées.
    """
    # === Initialisation de ROS2 ===
    
    rclpy.init(args=args)
    
    # === Gestion du Cycle de Vie du Nœud ===
    
    try:
        # Création et démarrage du nœud
        node = DistanceObjet()
        
        # Message de démarrage
        node.get_logger().info("=== Nœud de distance prêt - En attente de données ===")
        
        # Démarrage de la boucle d'exécution ROS2
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # Gestion de l'interruption utilisateur (Ctrl+C)
        print("\n=== Arrêt du nœud demandé par l'utilisateur ===")
        
    except Exception as e:
        # Gestion des erreurs inattendues
        print(f"Erreur inattendue dans le nœud de distance: {e}")
        
    finally:
        # === Nettoyage des Ressources ===
        
        # Destruction propre du nœud si il a été créé
        if 'node' in locals():
            node.get_logger().info("=== Arrêt du nœud de calcul de distance ===")
            node.destroy_node()
            
        # Arrêt propre de ROS2
        rclpy.shutdown()
        
        print("=== Nœud de distance arrêté proprement ===")


# === Point d'Entrée du Script ===

if __name__ == '__main__':
    main()