#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import time
import math

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

        # Création du subscriber pour la pose du contrôleur
        self.position_controleur_sub = self.create_subscription(
            Float32MultiArray, 
            '/valeur_effecteur', 
            self.position_controleur_sub_callback, 
            10
        )

        # Création d'un publisher pour la force du contrôleur
        self.guide_force_pub = self.create_publisher(
            Float32MultiArray,
            '/force_guide',
            10
        )

        # Création du subscriber pour le mode (position/vitesse)
        self.mode_sub = self.create_subscription(
            Float32MultiArray,
            '/Mode_Pose_Vitesse',
            self.mode_callback,
            10
        )

        # Création du subscriber pour la position de l'objet
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/fiole/pose',
            self.pose_callback,
            10
        )

        # Créer un subscriber pour l'état actuel des articulations
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Création du subscriber pour la distance entre parroies et robot
        self.distance_parroies_sub = self.create_subscription(
            Float32MultiArray,
            '/distance_robot_parroies',
            self.distance_parroies_callback,
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
        self.pince = 0.0

        # État des articulations (pour la callback joint_state)
        self.current_joint_positions = {}

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

        # Initialiser la position de l'objet
        self.pose_objet_x = 0.0
        self.pose_objet_y = 0.0
        self.pose_objet_z = 0.0
        self.pose_objet_qx = 0.0
        self.pose_objet_qy = 0.0
        self.pose_objet_qz = 0.0
        self.pose_objet_qw = 0.0
        
        # Initialiser les positions précédentes pour le calcul d'accélération
        self.distance_x_prev = 0.0
        self.distance_y_prev = 0.0
        self.distance_z_prev = 0.0

        # Initialiser les accélérations
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0

        # Initialiser les distances entre les parroies et l'effecteur du robot
        self.distance_parroies_x = 0.0
        self.distance_parroies_y = 0.0
        self.distance_parroies_z = 0.0

        # Initialiser les force de répulsion des parroies
        self.repulsion_parroies_x = 0.0
        self.repulsion_parroies_y = 0.0
        self.repulsion_parroies_z = 0.0

        # Initialiser la distance de sécurité entre robot et parroies
        self.distance_safe_parroies = 0.2
        
        # Variables pour le calcul du dt réel
        self.last_time = time.time()
        self.last_controller_update_time = time.time()
        
        # Timeout pour considérer les données du contrôleur comme obsolètes (en secondes)
        self.controller_data_timeout = 0.5

        # Initialiser la masse
        self.masse = 1.0
        self.masse_obj = 0.0
        
        # Initialiser les flags de contrôle
        self.robot_pose_received_ = False
        self.controller_pose_received_ = False
        self.controller_data_initialized_ = False  # Pour savoir si on a reçu au moins une donnée
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
        if len(msg.data) > 6:
            self.controleur_qw = float(msg.data[6])
        
        # Marquer que des données du contrôleur ont été reçues
        self.controller_pose_received_ = True
        self.controller_data_initialized_ = True
        self.last_controller_update_time = time.time()
        
        self.get_logger().debug(f"Position contrôleur reçue: [{self.controleur_x}, {self.controleur_y}, {self.controleur_z}] " +
                                f"[{self.controleur_qx}, {self.controleur_qy}, {self.controleur_qz}, {self.controleur_qw}]")

    def joint_state_callback(self, msg):
        """Callback pour recevoir l'état actuel des articulations"""
        # Mettre à jour l'état actuel des articulations
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
                
                # Si une des articulations correspond à la pince, la mettre à jour
                if 'fr3_finger_joint1' in name.lower() or 'fr3_finger_joint2' in name.lower():
                    self.pince = abs(msg.position[i])  # Valeur absolue de l'ouverture de la pince

    # Callback pour recevoir le mode (position/vitesse)
    def mode_callback(self, msg):
        """Callback pour recevoir le mode actuel"""
        if len(msg.data) < 2:
            self.get_logger().error(f"Message Mode_Pose_Vitesse invalide: attendu au moins 2 valeurs, reçu {len(msg.data)}")
            return
            
        # Mode Position = [1.0, 0.0, x, y], Mode Vitesse = [0.0, 1.0, x, y]
        self.in_pose_mode_ = bool(msg.data[0])  # 1.0 si en mode position
        
        self.get_logger().debug(f"Mode mis à jour: {'Position' if self.in_pose_mode_ else 'Vitesse'}")

    # Callback pour recevoir la distance entre les parroies et le robot
    def distance_parroies_callback(self, msg):
        """Callback pour recevoir la distance entre les parroies et le robot"""
        if len(msg.data) < 3:
            self.get_logger().error(f"Message Mode_Pose_Vitesse invalide: attendu au moins 3 valeurs, reçu {len(msg.data)}")
            return
            
        self.distance_parroies_x = (msg.data[0])
        self.distance_parroies_y = (msg.data[1])
        self.distance_parroies_z = (msg.data[2])

        if self.distance_parroies_x < self.distance_safe_parroies and self.controleur_x > 0.25:
            self.repulsion_parroies_x = - 1.0
        elif self.distance_parroies_x < self.distance_safe_parroies and self.controleur_x < 0.25:
            self.repulsion_parroies_x = 1.0
        else :
            self.repulsion_parroies_x = 0.0

        if self.distance_parroies_y < self.distance_safe_parroies and self.controleur_z > 0.05:
            self.repulsion_parroies_z = - 1.0
        elif self.distance_parroies_y < self.distance_safe_parroies and self.controleur_z < 0.05:
            self.repulsion_parroies_z = 1.0
        else :
            self.repulsion_parroies_z = 0.0

        if self.distance_parroies_z < self.distance_safe_parroies and self.controleur_y > -0.02:
            self.repulsion_parroies_y = - 1.0
        elif self.distance_parroies_z < self.distance_safe_parroies and self.controleur_y < -0.02:
            self.repulsion_parroies_y = 1.0
        else :
            self.repulsion_parroies_y = 0.0
        
        self.get_logger().debug(f"Distances publiées: suivant x: {self.distance_parroies_x} suivant y: {self.distance_parroies_y} suivant z: {self.distance_parroies_z}")
        self.get_logger().debug(f"Force de répulsion: suivant x: {self.repulsion_parroies_x} suivant y: {self.repulsion_parroies_y} suivant z: {self.repulsion_parroies_z}")

    # Callback pour recevoir la position de l'objet
    def pose_callback(self, msg):
        """Callback pour recevoir la position de l'objet"""
        # Extraire la position de l'objet du message
        self.pose_objet_x = float(msg.pose.position.x)
        self.pose_objet_y = float(msg.pose.position.y)
        self.pose_objet_z = float(msg.pose.position.z)
        
        # Extraire l'orientation (quaternion) et la convertir en angles d'Euler (simplification)
        # Pour une utilisation complète, il faudrait utiliser des fonctions de conversion quaternion -> Euler
        self.pose_objet_qx = float(msg.pose.orientation.x)
        self.pose_objet_qy = float(msg.pose.orientation.y)
        self.pose_objet_qz = float(msg.pose.orientation.z)
        self.pose_objet_qw = float(msg.pose.orientation.w)
        
        self.get_logger().debug(f"Position objet reçue: [{self.pose_objet_x}, {self.pose_objet_y}, {self.pose_objet_z}] " +
                            f"Orientation: [{self.pose_objet_qx}, {self.pose_objet_qy}, {self.pose_objet_qz}, {self.pose_objet_qw}]")

    def calcul_distance(self):
        distance = math.sqrt( (self.x - self.pose_objet_x)**2 + (self.y - self.pose_objet_y)**2 + (self.z - self.pose_objet_z)**2)
        if distance < 0.1 and self.pince <= 0.02:
            self.masse_obj = 0.2

    def calcul_accel(self):
        """
        Calcule l'accélération basée sur les positions actuelles et précédentes.
        Cette fonction est maintenant appelée dans send_force() pour garantir un calcul systématique.
        """
        current_time = time.time()
        
        # Vérifier si les données du contrôleur sont trop anciennes
        if not self.controller_data_initialized_ or (current_time - self.last_controller_update_time) > self.controller_data_timeout:
            # Données obsolètes ou inexistantes, mettre l'accélération à zéro
            self.accel_x = 0.0
            self.accel_y = 0.0
            self.accel_z = 0.0
            if self.count % 100 == 0:  # Log périodiquement
                self.get_logger().debug("Données du contrôleur obsolètes ou manquantes - Accélération mise à zéro")
            return self.accel_x, self.accel_y, self.accel_z
        
        # Calculer le dt réel depuis le dernier calcul
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Éviter les dt trop petits ou trop grands qui pourraient causer des problèmes
        if dt < 0.001:  # Moins de 1ms
            dt = 0.001
        elif dt > 1.0:  # Plus de 1 seconde
            dt = 1.0
        
        # Calculer les déplacements (différence entre position actuelle et position précédente)
        delta_x = self.controleur_x - self.distance_x_prev
        delta_y = self.controleur_y - self.distance_y_prev
        delta_z = self.controleur_z - self.distance_z_prev

        # Calculer les accélérations seulement si le déplacement est significatif
        if abs(delta_x) > 0.001 or abs(delta_y) > 0.001 or abs(delta_z) > 0.001:
            self.accel_x = delta_x / (dt**2)
            self.accel_y = delta_y / (dt**2)
            self.accel_z = delta_z / (dt**2)
        else:
            # Si le contrôleur est stationnaire (ou presque), l'accélération est nulle
            self.accel_x = 0.0
            self.accel_y = 0.0
            self.accel_z = 0.0

        # Limiter les accélérations
        if self.accel_x > 3:
            self.accel_x = 3.0
        elif self.accel_x < -3:
            self.accel_x = -3

        if self.accel_y > 3:
            self.accel_y = 3.0
        elif self.accel_y < -3:
            self.accel_y = -3

        if self.accel_z > 3:
            self.accel_z = 3.0
        elif self.accel_z < -3:
            self.accel_z = -3
        
        # Mettre à jour les positions précédentes
        self.distance_x_prev = self.controleur_x
        self.distance_y_prev = self.controleur_y
        self.distance_z_prev = self.controleur_z

        # Afficher l'information d'accélération périodiquement
        if self.count % 100 == 0:
            self.get_logger().debug(f"dt: {dt:.3f}s - Accélération suivant x: {self.accel_x:.2f}  y: {self.accel_y:.2f}  z: {self.accel_z:.2f}")
        
        return self.accel_x, self.accel_y, self.accel_z        

    def send_force(self):
        """
        Calcule l'accélération puis applique la formule F = m*a pour générer les forces.
        Cette fonction est appelée périodiquement par le timer à 10Hz.
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
            
            # Afficher l'état du mode tous les 100 cycles (environ 10 secondes à 10Hz)
            if self.count % 100 == 0:
                self.get_logger().debug("Mode Vitesse: Aucune force appliquée par guide_virtuel")
            
            return
        
        # CALCULER L'ACCÉLÉRATION À CHAQUE CYCLE
        self.calcul_accel()

        # Calculer la distance et mettre à jour la masse de l'objet
        self.calcul_distance()
        
        # Réinitialiser les forces
        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        self.force_rx = 0.0
        self.force_ry = 0.0
        self.force_rz = 0.0
        
        # Calcul de la force F = m*a
        self.force_x = - (self.masse + self.masse_obj) * self.accel_x + self.repulsion_parroies_x
        self.force_y = - (self.masse + self.masse_obj) * self.accel_y + self.repulsion_parroies_y
        self.force_z = - (self.masse + self.masse_obj) * self.accel_z + self.repulsion_parroies_z
        
        # Pour l'instant on ne prend pas en compte la rotation
        self.force_rx = 0.0
        self.force_ry = 0.0
        self.force_rz = 0.0

        # Créer et envoyer le message de force
        force_msg = Float32MultiArray()
        force_msg.data = [self.force_x, self.force_y, self.force_z, self.force_rx, self.force_ry, self.force_rz]
        self.guide_force_pub.publish(force_msg)
        
        # Afficher les forces périodiquement (tous les cycles dans votre version originale)
        if self.count % 100 == 0:
            self.get_logger().debug(f"Mode Position - Force envoyée: [{self.force_x:.3f}, {self.force_z:.3f}, {self.force_y:.3f}]")

def main(args=None):
    rclpy.init(args=args)
    node = GuideVirtuel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
