#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import time
import math

class ForcePosition(Node):
    def __init__(self):
        super().__init__('force_position')

        # Création du subscriber pour la pose du contrôleur
        self.position_controleur_sub = self.create_subscription(
            Float32MultiArray, 
            '/valeur_effecteur', 
            self.position_controleur_sub_callback, 
            10
        )

        # Création d'un publisher pour la force du contrôleur
        self.force_position_pub = self.create_publisher(
            Float32MultiArray,
            '/force_position',
            10
        )

        # Création du subscriber pour le mode (position/vitesse)
        self.mode_sub = self.create_subscription(
            Float32MultiArray,
            '/Mode_Pose_Vitesse',
            self.mode_callback,
            10
        )

        # Création du subscriber pour connaitre la distance entre le robot et l'objet
        self.distance_obj_sub= self.create_subscription(
            Float32MultiArray,
            '/distance_robot_objet',
            self.distance_obj_callback,
            10
        )

        # Créer un subscriber pour l'état actuel des articulations
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/NS_1/joint_states', # Si c'est pour le vrai robot
            #'/joint_states', # Si c'est pour la simulation
            self.joint_state_callback,
            10
        )

        # Initialiser la position de la pince
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

        # Variables pour l'orientation du contrôleur en angles d'Euler
        self.controleur_rx = 0.0  # Roll
        self.controleur_ry = 0.0  # Pitch
        self.controleur_rz = 0.0  # Yaw 

        # Initialiser la force du contrôleur
        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        self.force_rx = 0.0
        self.force_ry = 0.0
        self.force_rz = 0.0

        # Initialiser la position de l'objet
        self.distance_objet = 0.0

        # Initialiser les positions précédentes pour le calcul d'accélération
        self.distance_x_prev = 0.0
        self.distance_y_prev = 0.0
        self.distance_z_prev = 0.0
        self.speed_rx_prev = 0.0
        self.speed_ry_prev = 0.0
        self.speed_rz_prev = 0.0

        # Initialiser les vitesses
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0
        self.speed_rx = 0.0
        self.speed_ry = 0.0
        self.speed_rz = 0.0

        # Initialiser les accélérations
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.accel_rx = 0.0
        self.accel_ry = 0.0
        self.accel_rz = 0.0

        # Variables pour le calcul du dt réel
        self.last_time = time.time()
        self.last_controller_update_time = time.time()

        # Timeout pour considérer les données du contrôleur comme obsolètes (en secondes)
        self.controller_data_timeout = 0.5

        # Initialiser les masses
        self.masse_pince = 0.73
        self.masse_obj = 0.0
        self.force_distance_objet = 0.0

        # Initialiser la viscosité
        self.viscosite = 5.0
        self.viscosite_rot = 0.1
        self.frottement = 0.2
        self.couple_sec = 0.05

        # Inertie pour pince ouverte
        self.inertie_rx = 0.003
        self.inertie_ry = 0.0011
        self.inertie_rz = 0.0021

        # Inertie pour tube à essais
        self.inertie_tube_rx = 0.001877
        self.inertie_tube_ry = 0.001877
        self.inertie_tube_rz = 0.00010935

        # Initialiser les flags de contrôle
        self.controller_pose_received_ = False
        self.controller_data_initialized_ = False  # Pour savoir si on a reçu au moins une donnée
        self.in_pose_mode_ = True  # Par défaut, on est en mode position

        # Compteur pour les messages de debug
        self.count = 0

        # Créer un timer pour appeler périodiquement send_force
        self.timer = self.create_timer(0.1, self.send_force)  # Appel à 10Hz

        self.get_logger().info('Nœud de création de force_position démarré')

    def position_controleur_sub_callback(self, msg):
        """Callback pour recevoir la position du contrôleur"""
        # Vérifier que le message contient assez de données
        if len(msg.data) < 7:
            self.get_logger().error(f"Message valeur_effecteur invalide: attendu au moins 7 valeurs, reçu {len(msg.data)}")
            return

        # Mettre à jour la position du contrôleur
        self.controleur_x = float(msg.data[0])
        self.controleur_y = float(msg.data[1])
        self.controleur_z = float(msg.data[2])
        self.controleur_qx = float(msg.data[3])
        self.controleur_qy = float(msg.data[4])
        self.controleur_qz = float(msg.data[5])
        self.controleur_qw = float(msg.data[6])

        # Rotation de 22.5° autour de Y
        angle = math.pi / 8
        q_rot = (math.cos(angle/2), 0, math.sin(angle/2), 0)

        # Appliquer la rotation
        q_orig = (self.controleur_qw, self.controleur_qx, self.controleur_qy, self.controleur_qz)
        self.controleur_qw, self.controleur_qx, self.controleur_qy, self.controleur_qz = self.quaternion_multiply(q_orig, q_rot)

        # Convertir le quaternion en angles d'Euler pour les calculs d'accélération
        self.controleur_rx, self.controleur_ry, self.controleur_rz = self.quaternion_to_euler(
            self.controleur_qx, self.controleur_qy, self.controleur_qz, self.controleur_qw
        )

        # Marquer que des données du contrôleur ont été reçues
        self.controller_pose_received_ = True
        self.controller_data_initialized_ = True
        self.last_controller_update_time = time.time()        

        self.get_logger().debug(f"Position contrôleur reçue: [{self.controleur_x:.3f}, {self.controleur_y:.3f}, {self.controleur_z:.3f}] " +
                                f"Orientation Euler: [{self.controleur_rx:.3f}, {self.controleur_ry:.3f}, {self.controleur_rz:.3f}]")

    def joint_state_callback(self, msg):
        """Callback pour recevoir l'état actuel des articulations"""
        # Mettre à jour l'état actuel des articulations
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

                # Si une des articulations correspond à la pince, la mettre à jour
                if 'fr3_finger_joint1' in name.lower() or 'fr3_finger_joint2' in name.lower():
                    self.pince = abs(msg.position[i])  # Valeur absolue de l'ouverture de la pince

    def mode_callback(self, msg):
        """Callback pour recevoir le mode actuel"""
        if len(msg.data) < 2:
            self.get_logger().error(f"Message Mode_Pose_Vitesse invalide: attendu au moins 2 valeurs, reçu {len(msg.data)}")
            return

        # Mode Position = [1.0, 0.0, x, y], Mode Vitesse = [0.0, 1.0, x, y]
        self.in_pose_mode_ = bool(msg.data[0])  # 1.0 si en mode position

        self.get_logger().debug(f"Mode mis à jour: {'Position' if self.in_pose_mode_ else 'Vitesse'}")

    # Callback pour recevoir la distance entre le robot et l'objet
    def distance_obj_callback(self, msg):
        """Callback pour recevoir la distance entre le robot et l'objet"""
        if len(msg.data) < 4:
            self.get_logger().error(f"Message distance_robot_objet invalide: attendu au moins 4 valeurs, reçu {len(msg.data)}")
            return

        self.distance_objet = (msg.data[0])
        self.force_distance_objet = 0.0 #( 1/self.distance_objet ) / 5
        self.get_logger().debug(f"Distance entre le robot et l'objet : {self.distance_objet}")
        self.get_logger().debug(f"Force en fonction de la distance entre le robot et l'objet : {self.force_distance_objet}")

        if self.distance_objet < 0.1 and self.pince <= 0.02:
            self.masse_obj = 0.2
        
    def calcul_accel_trans(self):
        """
        Calcule l'accélération linéaire basée sur les positions actuelles et précédentes.
        """
        current_time = time.time()

        # Vérifier si les données du contrôleur sont trop anciennes
        if not self.controller_data_initialized_ or (current_time - self.last_controller_update_time) > self.controller_data_timeout:
            # Données obsolètes ou inexistantes, mettre l'accélération à zéro
            self.accel_x = 0.0
            self.accel_y = 0.0
            self.accel_z = 0.0

            self.speed_x = 0.0
            self.speed_y = 0.0
            self.speed_z = 0.0

            self.frottement = 0.0

            if self.count % 100 == 0:  # Log périodiquement
                self.get_logger().debug("Données du contrôleur obsolètes ou manquantes - Accélération et vitesse mise à zéro")
            return

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

        # Calculer les accélérations et les vitesses seulement si le déplacement est significatif
        if abs(delta_x) > 0.001 or abs(delta_y) > 0.001 or abs(delta_z) > 0.001:
            self.accel_x = delta_x / (dt**2)
            self.accel_y = delta_y / (dt**2)
            self.accel_z = delta_z / (dt**2)

            self.speed_x = delta_x / dt
            self.speed_y = delta_y / dt
            self.speed_z = delta_z / dt

        else:
            # Si le contrôleur est stationnaire (ou presque), l'accélération est nulle
            self.accel_x = 0.0
            self.accel_y = 0.0
            self.accel_z = 0.0

            self.speed_x = 0.0
            self.speed_y = 0.0
            self.speed_z = 0.0

            self.frottement = 0.0

        # Limiter les accélérations
        if self.accel_x > 3.0:
            self.accel_x = 3.0
        elif self.accel_x < -3.0:
            self.accel_x = -3.0

        if self.accel_y > 3.0:
            self.accel_y = 3.0
        elif self.accel_y < -3.0:
            self.accel_y = -3.0

        if self.accel_z > 3.0:
            self.accel_z = 3.0
        elif self.accel_z < -3.0:
            self.accel_z = -3.0

        # Mettre à jour les positions précédentes
        self.distance_x_prev = self.controleur_x
        self.distance_y_prev = self.controleur_y
        self.distance_z_prev = self.controleur_z

        # Afficher l'information d'accélération périodiquement
        if self.count % 100 == 0:
            self.get_logger().debug(f"dt: {dt:.3f}s - Accélération lin: [{self.accel_x:.2f}, {self.accel_y:.2f}, {self.accel_z:.2f}] ")
            self.get_logger().debug(f"dt: {dt:.3f}s - Vitesse lin: [{self.speed_x:.2f}, {self.speed_y:.2f}, {self.speed_z:.2f}]")

        return self.accel_x, self.accel_y, self.accel_z, self.speed_x, self.speed_y, self.speed_z

    def calcul_accel_rot(self):
        """
        Calcule l'accélération angulaire basée sur les rotations actuelles et précédentes.
        Utilise la différence de quaternions pour calculer la vitesse angulaire,
        puis sa dérivée pour l'accélération angulaire.
        """
        current_time = time.time()
        
        # Vérifier si les données du contrôleur sont trop anciennes
        if not self.controller_data_initialized_ or (current_time - self.last_controller_update_time) > self.controller_data_timeout:
            # Données obsolètes, mettre les vitesses et accélérations angulaires à zéro
            self.speed_rx = 0.0
            self.speed_ry = 0.0
            self.speed_rz = 0.0
            self.accel_rx = 0.0
            self.accel_ry = 0.0
            self.accel_rz = 0.0
            return
        
        # Calculer le dt réel depuis le dernier calcul
        dt = current_time - self.last_time
        
        # Éviter les dt trop petits ou trop grands
        if dt < 0.001:
            dt = 0.001
        elif dt > 1.0:
            dt = 1.0
        
        # Quaternion actuel du contrôleur
        q_current = (self.controleur_qw, self.controleur_qx, self.controleur_qy, self.controleur_qz)
        
        # Si c'est le premier appel, initialiser les quaternions précédents
        if not hasattr(self, 'controleur_qw_prev'):
            self.controleur_qw_prev = self.controleur_qw
            self.controleur_qx_prev = self.controleur_qx
            self.controleur_qy_prev = self.controleur_qy
            self.controleur_qz_prev = self.controleur_qz
            # Vitesses et accélérations nulles au premier appel
            self.speed_rx = 0.0
            self.speed_ry = 0.0
            self.speed_rz = 0.0
            
            self.accel_rx = 0.0
            self.accel_ry = 0.0
            self.accel_rz = 0.0

            self.couple_sec = 0.0
            return
        
        # Quaternion précédent
        q_prev = (self.controleur_qw_prev, self.controleur_qx_prev, self.controleur_qy_prev, self.controleur_qz_prev)
        
        # Calculer la différence de quaternion : q_delta = q_current * conjugate(q_prev)
        q_prev_conj = self.quaternion_conjugate(q_prev)
        q_delta = self.quaternion_multiply(q_current, q_prev_conj)
        
        # Extraire l'angle et l'axe de rotation de q_delta
        # q_delta = [w, x, y, z] où w = cos(angle/2)
        w_clamped = max(-1.0, min(1.0, q_delta[0]))
        angle = 2 * math.acos(w_clamped)
        
        # Si l'angle est très petit, pas de rotation significative
        if angle < 0.1:
            omega = [0.0, 0.0, 0.0]
            self.couple_sec = 0.0
        else:
            # Calculer l'axe de rotation normalisé
            sin_half_angle = math.sin(angle/2)
            if abs(sin_half_angle) < 1e-6:
                omega = [0.0, 0.0, 0.0]
            else:
                axis = [q_delta[1]/sin_half_angle, q_delta[2]/sin_half_angle, q_delta[3]/sin_half_angle]
                # Vitesse angulaire = angle / dt * axe
                omega = [angle / dt * a for a in axis]
        
        # Sauvegarder les vitesses angulaires précédentes pour le calcul d'accélération
        speed_rx_prev = self.speed_rx
        speed_ry_prev = self.speed_ry
        speed_rz_prev = self.speed_rz
        
        # Mettre à jour les vitesses angulaires actuelles
        self.speed_rx = omega[0]
        self.speed_ry = omega[1]
        self.speed_rz = omega[2]
        
        # Calculer l'accélération angulaire = (vitesse_actuelle - vitesse_précédente) / dt
        self.accel_rx = (self.speed_rx - speed_rx_prev) / dt
        self.accel_ry = (self.speed_ry - speed_ry_prev) / dt
        self.accel_rz = (self.speed_rz - speed_rz_prev) / dt

        # Limiter les vitesse angulaires pour éviter des valeurs extrêmes
        max_speed_rot = 2.0  # rad/s
        self.speed_rx = max(-max_speed_rot, min(max_speed_rot, self.speed_rx))
        self.speed_ry = max(-max_speed_rot, min(max_speed_rot, self.speed_ry))
        self.speed_rz = max(-max_speed_rot, min(max_speed_rot, self.speed_rz))
        
        # Limiter les accélérations angulaires pour éviter des valeurs extrêmes
        max_accel_rot = 1.0  # rad/s²
        self.accel_rx = max(-max_accel_rot, min(max_accel_rot, self.accel_rx))
        self.accel_ry = max(-max_accel_rot, min(max_accel_rot, self.accel_ry))
        self.accel_rz = max(-max_accel_rot, min(max_accel_rot, self.accel_rz))
        
        # Sauvegarder les quaternions actuels pour le prochain calcul
        self.controleur_qw_prev = self.controleur_qw
        self.controleur_qx_prev = self.controleur_qx
        self.controleur_qy_prev = self.controleur_qy
        self.controleur_qz_prev = self.controleur_qz
        
        # Affichage debug périodique
        if self.count % 100 == 0:
            self.get_logger().debug(f"Vitesse ang: [{self.speed_rx:.3f}, {self.speed_ry:.3f}, {self.speed_rz:.3f}] rad/s")
            self.get_logger().debug(f"Accél ang: [{self.accel_rx:.3f}, {self.accel_ry:.3f}, {self.accel_rz:.3f}] rad/s²")

    def send_force(self):
        """
        Calcule la formule F = m*a pour générer les forces.
        Cette fonction est appelée périodiquement par le timer à 10Hz.
        """
        # Incrémenter le compteur pour affichage périodique
        self.count += 1

        # Initialiser les forces à zéro
        force_msg = Float32MultiArray()
        force_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Zéro pour toutes les forces

        # Si on n'est pas en mode position, envoyer des forces nulles et quitter
        if not self.in_pose_mode_:
            # Envoyer le message de force (toutes à zéro)
            self.force_position_pub.publish(force_msg)

            # Afficher l'état du mode tous les 100 cycles (environ 10 secondes à 10Hz)
            if self.count % 100 == 0:
                self.get_logger().debug("Mode Vitesse: Aucune force appliquée par force_position")

            return

        # CALCULER L'ACCÉLÉRATION À CHAQUE CYCLE
        self.calcul_accel_trans()
        self.calcul_accel_rot()

        # Calcul de la force F = m*a + V*v + Fs
        self.force_x = - ( (self.masse_pince + self.masse_obj) * self.accel_x + self.viscosite * self.speed_x + self.frottement + self.force_distance_objet)
        self.force_y = - ( (self.masse_pince + self.masse_obj) * self.accel_y + self.viscosite * self.speed_y + self.frottement + self.force_distance_objet)
        self.force_z = - ( (self.masse_pince + self.masse_obj) * self.accel_z + self.viscosite * self.speed_z + self.frottement + self.force_distance_objet)

        # Calcul du couple C = I*a + V*v + Cs
        self.force_rx = - ( (self.inertie_rx + self.inertie_tube_rx) * self.accel_rx/10 + self.viscosite_rot * self.speed_rx/5 + self.couple_sec )
        self.force_ry = - ( (self.inertie_ry + self.inertie_tube_ry) * self.accel_ry/5 + self.viscosite_rot * self.speed_ry/2 + self.couple_sec )
        self.force_rz = - ( (self.inertie_rz + self.inertie_tube_rz) * self.accel_rz/25 + self.viscosite_rot * self.speed_rz/15 + self.couple_sec )

        # Limiter le couple
        if abs(self.force_rx) > 0.15:
            self.force_rx = 0.15

        if abs(self.force_ry) > 0.2:
            self.force_ry = 0.2

        if abs(self.force_rz) > 0.075:
            self.force_rz = 0.075

        # Créer et envoyer le message de force
        force_msg = Float32MultiArray()
        force_msg.data = [self.force_x, self.force_y, self.force_z, self.force_rx, self.force_ry, self.force_rz]
        self.force_position_pub.publish(force_msg)

        # Afficher les forces périodiquement (tous les cycles dans votre version originale)
        if self.count % 100 == 0:
            self.get_logger().debug(f"Mode Position - Force envoyée: [{self.force_x:.3f}, {self.force_y:.3f}, {self.force_z:.3f}] " +
                                   f"ang: [{self.force_rx:.3f}, {self.force_ry:.3f}, {self.force_rz:.3f}]")

    def quaternion_multiply(self, q1, q2):
        """
        Multiplie deux quaternions q1 et q2.
        Format des quaternions: (w, x, y, z)
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        return (
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        )

    def quaternion_to_euler(self, qx, qy, qz, qw):
        """
        Convertit un quaternion en angles d'Euler (roll, pitch, yaw).
        Implémente la conversion selon la convention ZYX (yaw, pitch, roll).
        """
        # Normaliser le quaternion
        norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        qw /= norm
        qx /= norm
        qy /= norm
        qz /= norm

        # Conversion en angles d'Euler
        # Roll (rotation autour de X)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (rotation autour de Y)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Utiliser 90 degrés si sinp est hors limites
        else:
            pitch = math.asin(sinp)

        # Yaw (rotation autour de Z)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if abs(pitch) > 1.2:
            # Gimbal lock imminent
            roll = 0.0
            yaw = 0.0
        return roll, pitch, yaw
    
    def quaternion_conjugate(self, q):
        """
        Calcule le conjugué d'un quaternion.
        Pour q = (w, x, y, z), le conjugué est (w, -x, -y, -z)
        """
        return (q[0], -q[1], -q[2], -q[3])


def main(args=None):
    rclpy.init(args=args)
    node = ForcePosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()