#!/usr/bin/env python3
"""
Module de calcul et génération de forces haptiques pour la téléopération du robot Franka FR3.

Ce module implémente un système de retour de force basé sur les lois de la physique (F = m*a)
pour créer un rendu haptique réaliste lors de la téléopération d'un robot Franka FR3 avec
un contrôleur haptique Desktop 6D de Haption.

Le système calcule les forces et couples nécessaires en fonction de:
- L'accélération du contrôleur haptique (inertie simulée)
- La vitesse du contrôleur (amortissement visqueux)
- Les frottements statiques et dynamiques
- La distance aux objets dans l'environnement

Auteur: Vincent Bassemayousse
Date: 07/10/2025
Version: 1.0
Licence: Apache 2.0
Projet: Téléopération robot Franka FR3 avec retour haptique
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import time
import math


class ForcePosition(Node):
    """
    Nœud ROS2 pour le calcul et la génération de forces haptiques.
    
    Ce nœud calcule les forces et couples à appliquer au contrôleur haptique
    en fonction de la position, vitesse et accélération du contrôleur, ainsi que
    de l'état du robot et de l'environnement.
    
    Le système utilise un modèle physique basé sur:
    - Forces linéaires: F = m*a + b*v + F_friction + F_distance
    - Couples angulaires: C = I*α + b*ω + C_friction
    
    Attributes:
        Subscribers:
            - /valeur_effecteur: Position et orientation du contrôleur haptique
            - /Mode_Pose_Vitesse: Mode de fonctionnement (position/vitesse)
            - /distance_robot_objet: Distance entre le robot et les objets
            - /NS_1/joint_states: État des articulations du robot
            
        Publishers:
            - /force_position: Forces et couples à appliquer au contrôleur
            
        Paramètres physiques:
            - Masses: Pince du robot et objets manipulés
            - Inerties: Moments d'inertie pour les rotations
            - Amortissement: Coefficients de viscosité
            - Frottements: Forces de frottement sec
    """
    
    def __init__(self):
        """
        Initialise le nœud de calcul des forces haptiques.
        
        Configure tous les subscribers, publishers, variables d'état et
        paramètres physiques du système de retour de force.
        """
        super().__init__('force_position')

        # ========================================
        # CONFIGURATION DES TOPICS ROS2
        # ========================================
        
        # Subscriber pour recevoir la pose du contrôleur haptique
        # Topic: /valeur_effecteur - Format: [x, y, z, qx, qy, qz, qw]
        self.position_controleur_sub = self.create_subscription(
            Float32MultiArray, 
            '/valeur_effecteur', 
            self.position_controleur_sub_callback, 
            10
        )

        # Publisher pour envoyer les forces au contrôleur haptique
        # Topic: /force_position - Format: [Fx, Fy, Fz, Tx, Ty, Tz]
        self.force_position_pub = self.create_publisher(
            Float32MultiArray,
            '/force_position',
            10
        )

        # Subscriber pour connaître le mode de fonctionnement
        # Topic: /Mode_Pose_Vitesse - Format: [mode_position, mode_vitesse, x, y]
        self.mode_sub = self.create_subscription(
            Float32MultiArray,
            '/Mode_Pose_Vitesse',
            self.mode_callback,
            10
        )

        # Subscriber pour la distance entre le robot et les objets
        # Topic: /distance_robot_objet - Format: [distance, ...]
        self.distance_obj_sub = self.create_subscription(
            Float32MultiArray,
            '/distance_robot_objet',
            self.distance_obj_callback,
            10
        )

        # Subscriber pour l'état des articulations du robot
        # Topic: /NS_1/joint_states (robot réel) ou /joint_states (simulation)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/NS_1/joint_states',  # Pour le robot réel
            # '/joint_states',     # Pour la simulation Gazebo
            self.joint_state_callback,
            10
        )

        # ========================================
        # VARIABLES D'ÉTAT DU SYSTÈME
        # ========================================
        
        # État de la pince du robot (ouverture en mètres)
        self.pince = 0.0
        
        # État actuel des articulations du robot
        self.current_joint_positions = {}

        # Position cartésienne du contrôleur haptique (en mètres)
        self.controleur_x = 0.0
        self.controleur_y = 0.0
        self.controleur_z = 0.0
        
        # Orientation du contrôleur en quaternions
        self.controleur_qx = 0.0
        self.controleur_qy = 0.0
        self.controleur_qz = 0.0
        self.controleur_qw = 1.0  # Quaternion d'identité (pas de rotation)

        # Orientation du contrôleur en angles d'Euler (radians)
        self.controleur_rx = 0.0  # Roll (rotation autour de X)
        self.controleur_ry = 0.0  # Pitch (rotation autour de Y)
        self.controleur_rz = 0.0  # Yaw (rotation autour de Z)

        # Forces et couples calculés à appliquer au contrôleur
        self.force_x = 0.0   # Force linéaire selon X (Newton)
        self.force_y = 0.0   # Force linéaire selon Y (Newton)
        self.force_z = 0.0   # Force linéaire selon Z (Newton)
        self.force_rx = 0.0  # Couple autour de X (Newton-mètre)
        self.force_ry = 0.0  # Couple autour de Y (Newton-mètre)
        self.force_rz = 0.0  # Couple autour de Z (Newton-mètre)

        # Distance entre le robot et les objets de l'environnement
        self.distance_objet = 0.0

        # ========================================
        # VARIABLES POUR LE CALCUL CINÉMATIQUE
        # ========================================
        
        # Positions précédentes pour le calcul de vitesse et accélération
        self.distance_x_prev = 0.0
        self.distance_y_prev = 0.0
        self.distance_z_prev = 0.0
        self.speed_rx_prev = 0.0
        self.speed_ry_prev = 0.0
        self.speed_rz_prev = 0.0

        # Vitesses linéaires calculées (m/s)
        self.speed_x = 0.0
        self.speed_y = 0.0
        self.speed_z = 0.0
        
        # Vitesses angulaires calculées (rad/s)
        self.speed_rx = 0.0
        self.speed_ry = 0.0
        self.speed_rz = 0.0

        # Accélérations linéaires calculées (m/s²)
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0
        
        # Accélérations angulaires calculées (rad/s²)
        self.accel_rx = 0.0
        self.accel_ry = 0.0
        self.accel_rz = 0.0

        # ========================================
        # GESTION DU TEMPS ET TIMEOUTS
        # ========================================
        
        # Timestamps pour le calcul du dt réel
        self.last_time = time.time()
        self.last_controller_update_time = time.time()

        # Timeout pour considérer les données du contrôleur comme obsolètes
        self.controller_data_timeout = 0.5  # 500ms

        # ========================================
        # PARAMÈTRES PHYSIQUES DU SYSTÈME
        # ========================================
        
        # Masses des éléments manipulés (kg)
        self.masse_pince = 0.73      # Masse de la pince du robot FR3
        self.masse_obj = 0.0         # Masse de l'objet saisi (mise à jour dynamiquement)
        self.force_distance_objet = 0.0  # Force liée à la proximité d'objets

        # Coefficients d'amortissement visqueux
        self.viscosite = 5.0         # Amortissement linéaire (N·s/m)
        self.viscosite_rot = 0.1     # Amortissement rotationnel (N·m·s/rad)
        
        # Coefficients de frottement
        self.frottement = 0.2        # Frottement sec linéaire (N)
        self.couple_sec = 0.05       # Frottement sec rotationnel (N·m)

        # Moments d'inertie de la pince ouverte (kg·m²)
        self.inertie_rx = 0.003      # Inertie autour de X
        self.inertie_ry = 0.0011     # Inertie autour de Y
        self.inertie_rz = 0.0021     # Inertie autour de Z

        # Moments d'inertie supplémentaires pour tube à essais (kg·m²)
        self.inertie_tube_rx = 0.001877    # Inertie tube autour de X
        self.inertie_tube_ry = 0.001877    # Inertie tube autour de Y
        self.inertie_tube_rz = 0.00010935  # Inertie tube autour de Z

        # Limite de couple pour le contrôleur (N·m)
        self.couple_max_x = 0.15
        self.couple_max_y = 0.2
        self.couple_max_z = 0.075

        # ========================================
        # FLAGS DE CONTRÔLE
        # ========================================
        
        # Indicateurs d'état des données reçues
        self.controller_pose_received_ = False     # Données de pose reçues
        self.controller_data_initialized_ = False  # Au moins une donnée reçue
        self.in_pose_mode_ = True                  # Mode position (True) ou vitesse (False)

        # Compteur pour les messages de debug périodiques
        self.count = 0

        # ========================================
        # TIMER PRINCIPAL
        # ========================================
        
        # Timer pour le calcul et envoi des forces à 10Hz
        self.timer = self.create_timer(0.1, self.send_force)

        self.get_logger().info('Nœud de création de force_position démarré')

    def position_controleur_sub_callback(self, msg):
        """
        Callback pour recevoir la position et orientation du contrôleur haptique.
        
        Traite les données de pose du contrôleur haptique, applique une rotation
        de calibration et convertit les quaternions en angles d'Euler pour les
        calculs ultérieurs.
        
        Args:
            msg (Float32MultiArray): Message contenant [x, y, z, qx, qy, qz, qw]
                - x, y, z: Position cartésienne en mètres
                - qx, qy, qz, qw: Quaternion d'orientation
        
        Note:
            Une rotation de 22.5° autour de l'axe Y est appliquée pour calibrer
            l'orientation du contrôleur avec le repère du robot.
        """
        # Vérification de la validité du message
        if len(msg.data) < 7:
            self.get_logger().error(f"Message valeur_effecteur invalide: attendu au moins 7 valeurs, reçu {len(msg.data)}")
            return

        # Extraction des données de position
        self.controleur_x = float(msg.data[0])
        self.controleur_y = float(msg.data[1])
        self.controleur_z = float(msg.data[2])
        
        # Extraction des données d'orientation (quaternion)
        self.controleur_qx = float(msg.data[3])
        self.controleur_qy = float(msg.data[4])
        self.controleur_qz = float(msg.data[5])
        self.controleur_qw = float(msg.data[6])

        # Application d'une rotation de calibration de 22.5° autour de Y
        # Cette rotation compense la différence d'orientation entre la base du contrôleur et l'effecteur
        angle = math.pi / 8  # 22.5° en radians
        q_rot = (math.cos(angle/2), 0, math.sin(angle/2), 0)  # Quaternion de rotation

        # Multiplication des quaternions pour appliquer la rotation
        q_orig = (self.controleur_qw, self.controleur_qx, self.controleur_qy, self.controleur_qz)
        self.controleur_qw, self.controleur_qx, self.controleur_qy, self.controleur_qz = self.quaternion_multiply(q_orig, q_rot)

        # Conversion du quaternion en angles d'Euler pour les calculs d'accélération
        self.controleur_rx, self.controleur_ry, self.controleur_rz = self.quaternion_to_euler(
            self.controleur_qx, self.controleur_qy, self.controleur_qz, self.controleur_qw
        )

        # Mise à jour des flags d'état
        self.controller_pose_received_ = True
        self.controller_data_initialized_ = True
        self.last_controller_update_time = time.time()

        # Log de debug périodique
        self.get_logger().debug(f"Position contrôleur reçue: [{self.controleur_x:.3f}, {self.controleur_y:.3f}, {self.controleur_z:.3f}] " +
                                f"Orientation Euler: [{self.controleur_rx:.3f}, {self.controleur_ry:.3f}, {self.controleur_rz:.3f}]")

    def joint_state_callback(self, msg):
        """
        Callback pour recevoir l'état actuel des articulations du robot.
        
        Met à jour l'état des articulations du robot, notamment l'ouverture
        de la pince qui influence les paramètres physiques du système.
        
        Args:
            msg (JointState): Message contenant l'état des articulations
                - name: Noms des articulations
                - position: Positions des articulations
                - velocity: Vitesses des articulations (optionnel)
                - effort: Efforts des articulations (optionnel)
        """
        # Mise à jour de l'état de toutes les articulations
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

                # Détection des articulations de la pince pour mise à jour
                if 'fr3_finger_joint1' in name.lower() or 'fr3_finger_joint2' in name.lower():
                    self.pince = abs(msg.position[i])  # Ouverture de la pince en valeur absolue

    def mode_callback(self, msg):
        """
        Callback pour recevoir le mode de fonctionnement du système.
        
        Détermine si le système fonctionne en mode position (avec retour de force)
        ou en mode vitesse (sans retour de force).
        
        Args:
            msg (Float32MultiArray): Message de mode
                - [1.0, 0.0, ...] : Mode position activé
                - [0.0, 1.0, ...] : Mode vitesse activé
        """
        if len(msg.data) < 2:
            self.get_logger().error(f"Message Mode_Pose_Vitesse invalide: attendu au moins 2 valeurs, reçu {len(msg.data)}")
            return

        # Mise à jour du mode (True = position, False = vitesse)
        self.in_pose_mode_ = bool(msg.data[0])

        self.get_logger().debug(f"Mode mis à jour: {'Position' if self.in_pose_mode_ else 'Vitesse'}")

    def distance_obj_callback(self, msg):
        """
        Callback pour recevoir la distance entre le robot et les objets.
        
        Met à jour la distance aux objets et calcule les forces de proximité.
        Détecte également la saisie d'objets pour ajuster la masse totale.
        
        Args:
            msg (Float32MultiArray): Message contenant la distance aux objets
                - [distance, ...] : Distance minimale aux objets en mètres
        """
        if len(msg.data) < 4:
            self.get_logger().error(f"Message distance_robot_objet invalide: attendu au moins 4 valeurs, reçu {len(msg.data)}")
            return

        # Mise à jour de la distance aux objets
        self.distance_objet = msg.data[0]
        
        # Calcul de la force de proximité (actuellement désactivée)
        self.force_distance_objet = 0.0  # Formule: (1/self.distance_objet) / 5
        
        # Logs de debug
        self.get_logger().debug(f"Distance entre le robot et l'objet : {self.distance_objet}")
        self.get_logger().debug(f"Force en fonction de la distance entre le robot et l'objet : {self.force_distance_objet}")

        # Détection de la saisie d'un objet
        # Si la distance est faible et la pince fermée, on considère qu'un objet est saisi
        if self.distance_objet < 0.1 and self.pince <= 0.02:
            self.masse_obj = 0.2  # Masse de l'objet saisi (kg)

    def calcul_accel_trans(self):
        """
        Calcule l'accélération et la vitesse linéaires du contrôleur haptique.
        
        Utilise la différence de position entre deux échantillons successifs
        pour calculer la vitesse, puis l'accélération par dérivation.
        
        Returns:
            tuple: (accel_x, accel_y, accel_z, speed_x, speed_y, speed_z)
                - Accélérations linéaires en m/s²
                - Vitesses linéaires en m/s
        
        Note:
            Les accélérations sont limitées à ±3 m/s² pour éviter les valeurs extrêmes.
            Un timeout de 500ms est appliqué pour détecter les données obsolètes.
        """
        current_time = time.time()

        # Vérification du timeout des données du contrôleur
        if not self.controller_data_initialized_ or (current_time - self.last_controller_update_time) > self.controller_data_timeout:
            # Données obsolètes : réinitialisation des valeurs
            self.accel_x = 0.0
            self.accel_y = 0.0
            self.accel_z = 0.0
            self.speed_x = 0.0
            self.speed_y = 0.0
            self.speed_z = 0.0
            self.frottement = 0.0

            if self.count % 100 == 0:  # Log périodique
                self.get_logger().debug("Données du contrôleur obsolètes ou manquantes - Accélération et vitesse mise à zéro")
            return

        # Calcul du pas de temps réel
        dt = current_time - self.last_time
        self.last_time = current_time

        # Limitation du dt pour éviter les valeurs aberrantes
        if dt < 0.001:      # Minimum 1ms
            dt = 0.001
        elif dt > 1.0:      # Maximum 1s
            dt = 1.0

        # Calcul des déplacements depuis la dernière mesure
        delta_x = self.controleur_x - self.distance_x_prev
        delta_y = self.controleur_y - self.distance_y_prev
        delta_z = self.controleur_z - self.distance_z_prev

        # Calcul de la vitesse et accélération si le déplacement est significatif
        if abs(delta_x) > 0.001 or abs(delta_y) > 0.001 or abs(delta_z) > 0.001:
            # Accélération = déplacement / dt²
            self.accel_x = delta_x / (dt**2)
            self.accel_y = delta_y / (dt**2)
            self.accel_z = delta_z / (dt**2)

            # Vitesse = déplacement / dt
            self.speed_x = delta_x / dt
            self.speed_y = delta_y / dt
            self.speed_z = delta_z / dt

        else:
            # Contrôleur stationnaire : accélération et vitesse nulles
            self.accel_x = 0.0
            self.accel_y = 0.0
            self.accel_z = 0.0
            self.speed_x = 0.0
            self.speed_y = 0.0
            self.speed_z = 0.0
            self.frottement = 0.0

        # Limitation des accélérations pour éviter les valeurs extrêmes
        max_accel = 3.0  # m/s²
        self.accel_x = max(-max_accel, min(max_accel, self.accel_x))
        self.accel_y = max(-max_accel, min(max_accel, self.accel_y))
        self.accel_z = max(-max_accel, min(max_accel, self.accel_z))

        # Mise à jour des positions précédentes pour le prochain calcul
        self.distance_x_prev = self.controleur_x
        self.distance_y_prev = self.controleur_y
        self.distance_z_prev = self.controleur_z

        # Affichage debug périodique
        if self.count % 100 == 0:
            self.get_logger().debug(f"dt: {dt:.3f}s - Accélération lin: [{self.accel_x:.2f}, {self.accel_y:.2f}, {self.accel_z:.2f}] ")
            self.get_logger().debug(f"dt: {dt:.3f}s - Vitesse lin: [{self.speed_x:.2f}, {self.speed_y:.2f}, {self.speed_z:.2f}]")

        return self.accel_x, self.accel_y, self.accel_z, self.speed_x, self.speed_y, self.speed_z

    def calcul_accel_rot(self):
        """
        Calcule l'accélération et la vitesse angulaires du contrôleur haptique.
        
        Utilise la différence de quaternions pour calculer la vitesse angulaire,
        puis sa dérivée temporelle pour obtenir l'accélération angulaire.
        
        La méthode utilise la formule:
        - ω = 2 * ln(q_current * q_previous^(-1)) / dt
        - α = (ω_current - ω_previous) / dt
        
        Note:
            Les vitesses angulaires sont limitées à ±2 rad/s
            Les accélérations angulaires sont limitées à ±1 rad/s²
        """
        current_time = time.time()
        
        # Vérification du timeout des données du contrôleur
        if not self.controller_data_initialized_ or (current_time - self.last_controller_update_time) > self.controller_data_timeout:
            # Données obsolètes : réinitialisation des valeurs angulaires
            self.speed_rx = 0.0
            self.speed_ry = 0.0
            self.speed_rz = 0.0
            self.accel_rx = 0.0
            self.accel_ry = 0.0
            self.accel_rz = 0.0
            return
        
        # Calcul du pas de temps réel
        dt = current_time - self.last_time
        
        # Limitation du dt pour éviter les valeurs aberrantes
        if dt < 0.001:
            dt = 0.001
        elif dt > 1.0:
            dt = 1.0
        
        # Quaternion actuel du contrôleur
        q_current = (self.controleur_qw, self.controleur_qx, self.controleur_qy, self.controleur_qz)
        
        # Initialisation au premier appel
        if not hasattr(self, 'controleur_qw_prev'):
            self.controleur_qw_prev = self.controleur_qw
            self.controleur_qx_prev = self.controleur_qx
            self.controleur_qy_prev = self.controleur_qy
            self.controleur_qz_prev = self.controleur_qz
            
            # Valeurs initiales nulles
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
        
        # Calcul de la différence de quaternion : q_delta = q_current * q_prev^(-1)
        q_prev_conj = self.quaternion_conjugate(q_prev)
        q_delta = self.quaternion_multiply(q_current, q_prev_conj)
        
        # Extraction de l'angle de rotation du quaternion différence
        # Pour q_delta = [w, x, y, z], l'angle est θ = 2 * arccos(|w|)
        w_clamped = max(-1.0, min(1.0, q_delta[0]))
        angle = 2 * math.acos(w_clamped)
        
        # Calcul de la vitesse angulaire
        if angle < 0.1:  # Rotation négligeable
            omega = [0.0, 0.0, 0.0]
            self.couple_sec = 0.0
        else:
            # Calcul de l'axe de rotation normalisé
            sin_half_angle = math.sin(angle/2)
            if abs(sin_half_angle) < 1e-6:
                omega = [0.0, 0.0, 0.0]
            else:
                axis = [q_delta[1]/sin_half_angle, q_delta[2]/sin_half_angle, q_delta[3]/sin_half_angle]
                # Vitesse angulaire = angle / dt * axe
                omega = [angle / dt * a for a in axis]
        
        # Sauvegarde des vitesses précédentes pour le calcul d'accélération
        speed_rx_prev = self.speed_rx
        speed_ry_prev = self.speed_ry
        speed_rz_prev = self.speed_rz
        
        # Mise à jour des vitesses angulaires actuelles
        self.speed_rx = omega[0]
        self.speed_ry = omega[1]
        self.speed_rz = omega[2]
        
        # Calcul de l'accélération angulaire = (ω_current - ω_previous) / dt
        self.accel_rx = (self.speed_rx - speed_rx_prev) / dt
        self.accel_ry = (self.speed_ry - speed_ry_prev) / dt
        self.accel_rz = (self.speed_rz - speed_rz_prev) / dt

        # Limitation des vitesses angulaires
        max_speed_rot = 2.0  # rad/s
        self.speed_rx = max(-max_speed_rot, min(max_speed_rot, self.speed_rx))
        self.speed_ry = max(-max_speed_rot, min(max_speed_rot, self.speed_ry))
        self.speed_rz = max(-max_speed_rot, min(max_speed_rot, self.speed_rz))
        
        # Limitation des accélérations angulaires
        max_accel_rot = 1.0  # rad/s²
        self.accel_rx = max(-max_accel_rot, min(max_accel_rot, self.accel_rx))
        self.accel_ry = max(-max_accel_rot, min(max_accel_rot, self.accel_ry))
        self.accel_rz = max(-max_accel_rot, min(max_accel_rot, self.accel_rz))
        
        # Sauvegarde des quaternions pour le prochain calcul
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
        Calcule et envoie les forces de retour haptique pour la téléopération.
        
        Cette méthode implémente le contrôle de force basé sur la loi fondamentale F = m*a
        pour générer les forces et couples de retour haptique. Elle est appelée périodiquement
        par un timer à 10Hz pour assurer une mise à jour en temps réel des forces.
        
        Le calcul des forces suit le modèle physique:
        - Forces linéaires: F = m*a + v*v + Fs (masse, viscosite, frottement)
        - Couples rotationnels: C = I*α + v*ω + Cs (inertie, amortissement, couple sec)
        
        Les forces sont limitées pour éviter les mouvements brusques et assurer la sécurité
        de l'opérateur lors de l'utilisation du contrôleur haptique.
        
        Returns:
            None: Les forces sont directement publiées sur le topic ROS2
        
        Note:
            - En mode vitesse, toutes les forces sont mises à zéro
            - En mode position, les forces sont calculées selon le modèle physique
            - Les forces sont limitées par sécurité pour éviter les chocs
        """
        # Incrémenter le compteur pour l'affichage périodique des logs
        self.count += 1

        # Initialiser le message de force avec des valeurs nulles
        # Format: [force_x, force_y, force_z, couple_rx, couple_ry, couple_rz]
        force_msg = Float32MultiArray()
        force_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Zéro pour toutes les forces

        # Vérifier si on est en mode position (requis pour le contrôle de force)
        if not self.in_pose_mode_:
            # En mode vitesse, envoyer uniquement des forces nulles pour éviter les interférences
            self.force_position_pub.publish(force_msg)

            # Afficher l'état du mode tous les 100 cycles (environ 10 secondes à 10Hz)
            if self.count % 100 == 0:
                self.get_logger().debug("Mode Vitesse: Aucune force appliquée par force_position")

            return

        # ========== CALCUL DES ACCÉLÉRATIONS ==========
        # Mettre à jour les accélérations linéaires et angulaires pour ce cycle
        self.calcul_accel_trans()  # Calcul des accélérations linéaires (x, y, z)
        self.calcul_accel_rot()    # Calcul des accélérations angulaires (rx, ry, rz)

        # ========== CALCUL DES FORCES LINÉAIRES ==========
        # Modèle physique: F = -[m*a + v*v + Fs + F_distance]
        # Le signe négatif assure que la force s'oppose au mouvement (retour haptique)
        
        # Force en X (direction avant/arrière)
        self.force_x = -((self.masse_pince + self.masse_obj) * self.accel_x +     # Composante inertielle
                        self.viscosite * self.speed_x +                          # Amortissement visqueux
                        self.frottement +                                        # Frottement sec
                        self.force_distance_objet)                               # Force liée à la distance aux objets

        # Force en Y (direction gauche/droite) - même modèle
        self.force_y = -((self.masse_pince + self.masse_obj) * self.accel_y +
                        self.viscosite * self.speed_y +
                        self.frottement +
                        self.force_distance_objet)

        # Force en Z (direction haut/bas) - même modèle
        self.force_z = -((self.masse_pince + self.masse_obj) * self.accel_z +
                        self.viscosite * self.speed_z +
                        self.frottement +
                        self.force_distance_objet)

        # ========== CALCUL DES COUPLES ROTATIONNELS ==========
        # Modèle physique: C = -[I*α + v*ω + Cs]
        # Les facteurs de division (10, 5, 25) sont des gains d'ajustement empiriques
        
        # Couple autour de X (roulis/roll) - facteur 10 pour l'accélération, 5 pour la vitesse
        self.force_rx = -((self.inertie_rx + self.inertie_tube_rx) * self.accel_rx/10 +
                        self.viscosite_rot * self.speed_rx/5 +
                        self.couple_sec)

        # Couple autour de Y (tangage/pitch) - facteur 5 pour l'accélération, 2 pour la vitesse
        self.force_ry = -((self.inertie_ry + self.inertie_tube_ry) * self.accel_ry/5 +
                        self.viscosite_rot * self.speed_ry/2 +
                        self.couple_sec)

        # Couple autour de Z (lacet/yaw) - facteur 25 pour l'accélération, 15 pour la vitesse
        self.force_rz = -((self.inertie_rz + self.inertie_tube_rz) * self.accel_rz/25 +
                        self.viscosite_rot * self.speed_rz/15 +
                        self.couple_sec)

        # ========== LIMITATION DES COUPLES POUR LA SÉCURITÉ ==========
        # Limiter le couple en X pour éviter les mouvements brusques en roulis
        if abs(self.force_rx) > self.couple_max_x:
            self.force_rx = self.couple_max_x if self.force_rx > 0 else -self.couple_max_x

        # Limiter le couple en Y pour éviter les mouvements brusques en tangage
        if abs(self.force_ry) > self.couple_max_y:
            self.force_ry = self.couple_max_y if self.force_ry > 0 else -self.couple_max_y

        # Limiter le couple en Z pour éviter les mouvements brusques en lacet
        if abs(self.force_rz) > self.couple_max_z:
            self.force_rz = self.couple_max_z if self.force_rz > 0 else -self.couple_max_z

        # ========== PUBLICATION DES FORCES ==========
        # Créer et envoyer le message de force au contrôleur haptique
        force_msg = Float32MultiArray()
        force_msg.data = [self.force_x, self.force_y, self.force_z, 
                        self.force_rx, self.force_ry, self.force_rz]
        self.force_position_pub.publish(force_msg)

        # Afficher les forces périodiquement pour le débogage (tous les 100 cycles = 10s)
        if self.count % 100 == 0:
            self.get_logger().debug(f"Mode Position - Force envoyée: [{self.force_x:.3f}, {self.force_y:.3f}, {self.force_z:.3f}] " +
                                f"ang: [{self.force_rx:.3f}, {self.force_ry:.3f}, {self.force_rz:.3f}]")

    def quaternion_multiply(self, q1, q2):
        """
        Multiplie deux quaternions selon la formule de multiplication des quaternions.
        
        Cette opération est essentielle pour la composition de rotations dans l'espace 3D.
        La multiplication des quaternions n'est pas commutative (q1*q2 ≠ q2*q1).
        
        Args:
            q1 (tuple): Premier quaternion au format (w, x, y, z)
            q2 (tuple): Deuxième quaternion au format (w, x, y, z)
            
        Returns:
            tuple: Quaternion résultant de la multiplication (w, x, y, z)
            
        Note:
            La formule utilisée est la multiplication standard des quaternions:
            w = w1*w2 - x1*x2 - y1*y2 - z1*z2
            x = w1*x2 + x1*w2 + y1*z2 - z1*y2
            y = w1*y2 - x1*z2 + y1*w2 + z1*x2
            z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        """
        # Extraire les composantes des quaternions
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        # Calculer les composantes du quaternion résultant
        # selon la formule de multiplication des quaternions
        return (
            w1*w2 - x1*x2 - y1*y2 - z1*z2,  # Composante scalaire (w)
            w1*x2 + x1*w2 + y1*z2 - z1*y2,  # Composante vectorielle i (x)
            w1*y2 - x1*z2 + y1*w2 + z1*x2,  # Composante vectorielle j (y)
            w1*z2 + x1*y2 - y1*x2 + z1*w2   # Composante vectorielle k (z)
        )

    def quaternion_to_euler(self, qx, qy, qz, qw):
        """
        Convertit un quaternion en angles d'Euler (roll, pitch, yaw).
        
        Cette fonction implémente la conversion selon la convention ZYX (yaw-pitch-roll)
        avec une gestion spéciale du gimbal lock pour éviter les singularités.
        
        Args:
            qx (float): Composante x du quaternion
            qy (float): Composante y du quaternion
            qz (float): Composante z du quaternion
            qw (float): Composante w (scalaire) du quaternion
            
        Returns:
            tuple: Angles d'Euler (roll, pitch, yaw) en radians
            
        Note:
            - Roll: rotation autour de l'axe X (inclinaison latérale)
            - Pitch: rotation autour de l'axe Y (inclinaison avant/arrière)
            - Yaw: rotation autour de l'axe Z (rotation horizontale)
            - Gestion du gimbal lock pour pitch > 1.2 radians (~69°)
        """
        # ========== NORMALISATION DU QUATERNION ==========
        # Normaliser le quaternion pour éviter les erreurs de calcul
        norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        qw /= norm
        qx /= norm
        qy /= norm
        qz /= norm

        # ========== CONVERSION EN ANGLES D'EULER ==========
        
        # Calcul du Roll (rotation autour de X)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Calcul du Pitch (rotation autour de Y)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            # Gestion du cas limite où sinp sort de [-1, 1]
            pitch = math.copysign(math.pi / 2, sinp)  # Utiliser ±90° selon le signe
        else:
            pitch = math.asin(sinp)

        # Calcul du Yaw (rotation autour de Z)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # ========== GESTION DU GIMBAL LOCK ==========
        # Si le pitch est proche de ±90°, forcer roll et yaw à zéro
        # pour éviter les singularités de représentation
        if abs(pitch) > 1.2:  # Environ 69° en radians
            roll = 0.0
            yaw = 0.0
            
        return roll, pitch, yaw

    def quaternion_conjugate(self, q):
        """
        Calcule le conjugué d'un quaternion.
        
        Le conjugué d'un quaternion est utilisé pour inverser une rotation.
        Pour un quaternion unitaire, le conjugué est égal à l'inverse.
        
        Args:
            q (tuple): Quaternion d'entrée au format (w, x, y, z)
            
        Returns:
            tuple: Quaternion conjugué au format (w, -x, -y, -z)
            
        Note:
            Le conjugué d'un quaternion q = (w, x, y, z) est q* = (w, -x, -y, -z)
            Cette opération est utile pour calculer la rotation inverse.
        """
        # Retourner le conjugué: même partie scalaire, parties vectorielles opposées
        return (q[0], -q[1], -q[2], -q[3])


def main(args=None):
    """
    Fonction principale pour lancer le noeud ROS2 ForcePosition.
    
    Cette fonction initialise le système ROS2, crée une instance du noeud
    ForcePosition et lance la boucle d'exécution. Elle gère également
    la destruction propre du noeud lors de l'arrêt.
    
    Args:
        args (list, optional): Arguments de ligne de commande. Défaut: None
        
    Note:
        Cette fonction est appelée automatiquement si le script est exécuté
        directement (not imported as module).
    """
    # Initialiser le système ROS2
    rclpy.init(args=args)
    
    # Créer une instance du noeud ForcePosition
    node = ForcePosition()
    
    # Lancer la boucle d'exécution ROS2 (bloquante)
    rclpy.spin(node)
    
    # Nettoyer et détruire le noeud à la fin
    node.destroy_node()
    
    # Arrêter le système ROS2
    rclpy.shutdown()

if __name__ == '__main__':
    # Lancer la fonction principale si le script est exécuté directement
    main()