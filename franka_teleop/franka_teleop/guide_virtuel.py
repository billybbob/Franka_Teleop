#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

class GuideVirtuel(Node):
    def __init__(self):
        super().__init__('guide_virtuel')

        # Création du subscriber pour la pose de l'effecteur du robot
        self.position_effecteur_robot_sub = self.create_subscription(
            TFMessage, 
            '/position_effecteur_robot', 
            self.position_effecteur_robot_callback, 
            10
        )

        # Création d'un publisher pour la force du contrôleur
        self.guide_force_pub = self.create_publisher(
            Float32MultiArray,
            '/force_guide',
            10
        )
        
        # Création du subscriber pour la pose du contrôleur
        self.position_controleur_sub = self.create_subscription(
            Float32MultiArray, 
            '/valeur_effecteur', 
            self.position_controleur_sub_callback, 
            10
        )

        # Création du subscriber pour le mode (position/vitesse)
        self.mode_sub = self.create_subscription(
            Float32MultiArray,
            '/Mode_Pose_Vitesse',
            self.mode_callback,
            10
        )

        # Initialiser la position du robot
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0  # Quaternion d'identité (pas de rotation)

        # Initialiser la position du contrôleur
        self.controleur_x = 0.0
        self.controleur_y = 0.0
        self.controleur_z = 0.0
        self.controleur_qx = 0.0
        self.controleur_qy = 0.0
        self.controleur_qz = 0.0
        self.controleur_qw = 1.0  # Quaternion d'identité (pas de rotation)

        # Initialiser la force du contrôleur
        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        self.force_rx = 0.0
        self.force_ry = 0.0
        self.force_rz = 0.0

        # Initialiser la position de l'objet à récupérer
        self.pose_objet_x = 0.775
        self.pose_objet_y = 0.0
        self.pose_objet_z = 0.5
        self.pose_objet_rx = 0.0
        self.pose_objet_ry = 0.0
        self.pose_objet_rz = 0.0
        
        # Initialiser les distances précédentes
        self.distance_x_prev = 0.0
        self.distance_y_prev = 0.0
        self.distance_z_prev = 0.0
        
        # Initialiser les flags de contrôle
        self.robot_pose_received_ = False
        self.controller_pose_received_ = True  # Pour simplifier, on considère que le contrôleur est toujours disponible
        self.in_pose_mode_ = True  # Par défaut, on est en mode position
        
        # Compteur pour les messages de debug
        self.count = 0
        
        # Créer un timer pour appeler périodiquement send_force
        self.timer = self.create_timer(0.1, self.send_force)  # Appel à 10Hz
        
        self.get_logger().info('Nœud de création de guide virtuel démarré')

    # Callback pour recevoir la position de l'effecteur du robot
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
        
        self.robot_pose_received_ = True
        
        self.get_logger().debug(f"Position robot reçue: [{self.x}, {self.y}, {self.z}] " +
                                f"[{self.qx}, {self.qy}, {self.qz}, {self.qw}]")
        
    # Callback pour recevoir la position du contrôleur
    def position_controleur_sub_callback(self, msg):
        """Callback pour recevoir la position du contrôleur"""

        # Vérifier que le message contient assez de données (au moins 6 pour les vitesses cartésiennes)
        if len(msg.data) < 6:
            self.get_logger().error(f"Message valeur_effecteur invalide: attendu au moins 6 valeurs, reçu {len(msg.data)}")
            return
        
        # Mettre à jour la position du contrôleur
        self.controleur_x = float(msg.data[0])
        self.controleur_y = float(msg.data[1])
        self.controleur_z = float(msg.data[2])
        self.controleur_qx = float(msg.data[3])
        self.controleur_qy = float(msg.data[4])
        self.controleur_qz = float(msg.data[5])
        self.controleur_qw = float(msg.data[6])
        
        self.get_logger().debug(f"Position contrôleur reçue: [{self.controleur_x}, {self.controleur_y}, {self.controleur_z}] " +
                                f"[{self.controleur_qx}, {self.controleur_qy}, {self.controleur_qz}, {self.controleur_qw}]")

    # Callback pour recevoir le mode (position/vitesse)
    def mode_callback(self, msg):
        """Callback pour recevoir le mode actuel"""
        if len(msg.data) < 2:
            self.get_logger().error(f"Message Mode_Pose_Vitesse invalide: attendu au moins 2 valeurs, reçu {len(msg.data)}")
            return
            
        # Mode Position = [1.0, 0.0, x, y], Mode Vitesse = [0.0, 1.0, x, y]
        self.in_pose_mode_ = bool(msg.data[0])  # 1.0 si en mode position
        
        self.get_logger().debug(f"Mode mis à jour: {'Position' if self.in_pose_mode_ else 'Vitesse'}")

    def send_force(self):
        """
        Calcul la distance entre l'effecteur du robot et l'objet, puis applique une force si la distance
        actuelle est plus grande que la précédente, donc lorsque l'on s'éloigne.
        Les forces sont envoyées uniquement en mode position.
        """
        if not self.robot_pose_received_:
            return
            
        # Incrémenter le compteur pour affichage périodique
        self.count += 1
        
        # Initialiser les forces à zéro
        force_msg = Float32MultiArray()
        force_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Zéro pour toutes les forces
        
        # Si on n'est pas en mode position, envoyer des forces nulles et quitter
        if not self.in_pose_mode_:
            # Envoyer le message de force (toutes à zéro)
            self.guide_force_pub.publish(force_msg)
            
            # Afficher l'état du mode tous les 10 cycles (environ 1 seconde)
            if self.count % 10 == 0:
                self.get_logger().debug("Mode Vitesse: Aucune force appliquée par guide_virtuel")
            
            return
            
        # Si on est en mode position, continuer avec le calcul des forces
        # Calculer les distances actuelles
        distance_x = self.x - self.pose_objet_x
        distance_y = self.y - self.pose_objet_y
        distance_z = self.z - self.pose_objet_z

        if self.count % 10 == 0:  # Afficher seulement tous les 10 cycles pour réduire le spam
            self.get_logger().debug(f"Distance calculée: [{distance_x}, {distance_y}, {distance_z}]")
            self.get_logger().debug(f"Distance précédente: [{self.distance_x_prev}, {self.distance_y_prev}, {self.distance_z_prev}]")
        
        # Réinitialiser les forces
        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        self.force_rx = 0.0
        self.force_ry = 0.0
        self.force_rz = 0.0
        
        # Appliquer une force si on s'éloigne de l'objet
        if abs(distance_x) > abs(self.distance_x_prev):
            self.force_x = abs(distance_x) * 3.0
        
        if abs(distance_y) > abs(self.distance_y_prev):
            self.force_y = abs(distance_y) * 8.0
        
        if abs(distance_z) > abs(self.distance_z_prev):
            self.force_z = abs(distance_z) * 8.0
        
        # Pour l'instant on ne prend pas en compte la rotation
        self.force_rx = 0.0
        self.force_ry = 0.0
        self.force_rz = 0.0
        
        # Mettre à jour les distances précédentes
        self.distance_x_prev = distance_x
        self.distance_y_prev = distance_y
        self.distance_z_prev = distance_z

        # Valeurs max des forces
        if self.force_x > 2:
            self.force_x = 2.0

        if self.force_y > 2:
            self.force_y = 2.0

        if self.force_z > 2:
            self.force_z = 2.0
        
        # Modifier signe de force suivant position du contrôleur
        if (self.controleur_x - 0.25) > 0:
            self.force_x = - self.force_x

        if (self.controleur_y + 0.02) > 0:
            self.force_y = - self.force_y

        if (self.controleur_z - 0.05) > 0:
            self.force_z = - self.force_z

        # Créer et envoyer le message de force
        force_msg = Float32MultiArray()
        force_msg.data = [self.force_x, self.force_y, self.force_z, self.force_rx, self.force_ry, self.force_rz]
        self.guide_force_pub.publish(force_msg)
        
        if self.count % 10 == 0:  # Afficher seulement tous les 10 cycles
            self.get_logger().debug(f"Mode Position - Force envoyée: [{self.force_x}, {self.force_y}, {self.force_z}]")

def main(args=None):
    rclpy.init(args=args)
    node = GuideVirtuel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()