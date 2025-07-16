#!/usr/bin/env python3
"""
Module de téléopération haptique pour robot Franka FR3
=====================================================

Ce module implémente un nœud ROS2 pour contrôler un robot Franka FR3 via un contrôleur haptique
Desktop 6D de Haption. Il gère le retour de force en fonction de la position du robot et des
limites de sécurité définies.

Fonctionnalités principales:
- Calcul du retour de force basé sur la position et les limites articulaires
- Support des modes position et vitesse
- Amortissement adaptatif pour stabiliser le contrôle
- Gestion des collisions et détection d'objets
- Conversion quaternion vers angles d'Euler
- Calcul de vitesse et d'accélération en temps réel

Auteur: Vincent Bassemayousse
Date: 07/10/2025
Version: 1.0
Licence: Apache 2.0
Dépendances: ROS2 Humble, Gazebo Fortress
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import math
import time

class Joystick(Node):
    """
    Nœud ROS2 pour la téléopération haptique du robot Franka FR3.
    
    Ce nœud gère la communication entre le contrôleur haptique Desktop 6D et le robot,
    en calculant les forces de retour appropriées basées sur la position du robot,
    les limites de sécurité et les objets environnants.
    
    Attributes:
        force_joystick_pub: Publisher pour les forces du contrôleur
        position_controleur_sub: Subscriber pour la position du contrôleur
        mode_sub: Subscriber pour le mode de contrôle (position/vitesse)
        distance_obj_sub: Subscriber pour la distance aux objets
        joint_state_sub: Subscriber pour l'état des articulations
        
    Topics:
        Publications:
            - /force_joystick: Forces à appliquer au contrôleur haptique
            
        Subscriptions:
            - /valeur_effecteur: Position et orientation du contrôleur
            - /Mode_Pose_Vitesse: Mode de contrôle actuel
            - /distance_robot_objet: Distance aux objets proches
            - /NS_1/joint_states: État des articulations du robot
    """

    def __init__(self):
        """
        Initialise le nœud Joystick et configure tous les publishers/subscribers.
        
        Configure également tous les paramètres de contrôle, les limites de sécurité,
        et les constantes physiques utilisées pour le calcul des forces.
        """
        super().__init__('joystick')

        # ============================================================================
        # CONFIGURATION DES TOPICS ROS2
        # ============================================================================
        
        # Création d'un publisher pour envoyer les forces au contrôleur haptique
        self.force_joystick_pub = self.create_publisher(
            Float32MultiArray,
            '/force_joystick',  # Topic de sortie des forces
            10
        )
        
        # Création du subscriber pour recevoir la position du contrôleur haptique
        self.position_controleur_sub = self.create_subscription(
            Float32MultiArray, 
            '/valeur_effecteur',  # Topic d'entrée de la position du contrôleur
            self.position_controleur_sub_callback, 
            10
        )
        
        # Création du subscriber pour recevoir le mode de contrôle
        self.mode_sub = self.create_subscription(
            Float32MultiArray,
            '/Mode_Pose_Vitesse',  # Topic pour switcher entre mode position et vitesse
            self.mode_callback,
            10
        )

        # Création du subscriber pour la distance aux objets (détection de collision)
        self.distance_obj_sub= self.create_subscription(
            Float32MultiArray,
            '/distance_robot_objet',  # Topic pour la distance aux objets
            self.distance_obj_callback,
            10
        )

        # Création du subscriber pour l'état des articulations du robot
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/NS_1/joint_states',   # Topic pour le robot réel
            # '/joint_states',       # Topic pour la simulation
            self.joint_state_callback,
            10
        )

        # ============================================================================
        # VARIABLES D'ÉTAT DU CONTRÔLEUR HAPTIQUE
        # ============================================================================
        
        # Position cartésienne du contrôleur (en mètres)
        self.controleur_x = 0.0
        self.controleur_y = 0.0
        self.controleur_z = 0.0
        
        # Orientation du contrôleur en quaternion (qw, qx, qy, qz)
        self.controleur_qx = 0.0
        self.controleur_qy = 0.0
        self.controleur_qz = 0.0
        self.controleur_qw = 1.0  # Quaternion d'identité (pas de rotation)

        # ============================================================================
        # VARIABLES D'ÉTAT DU ROBOT ET DE L'ENVIRONNEMENT
        # ============================================================================
        
        # État de la pince du robot (ouverture en mètres)
        self.pince = 0.0

        # Distance aux objets proches et force associée
        self.distance_objet = 0.0
        self.force_distance_objet = 0.0

        # État actuel des articulations du robot (dictionnaire nom -> position)
        self.current_joint_positions = {}
        
        # Angles d'Euler correspondant au quaternion du contrôleur
        self.euler_x = 0.0  # Roll (rotation autour de X)
        self.euler_y = 0.0  # Pitch (rotation autour de Y)
        self.euler_z = 0.0  # Yaw (rotation autour de Z)

        # ============================================================================
        # VARIABLES DE CALCUL DES FORCES
        # ============================================================================
        
        # Forces calculées à appliquer au contrôleur haptique
        self.force_x = 0.0   # Force en translation X (N)
        self.force_y = 0.0   # Force en translation Y (N)
        self.force_z = 0.0   # Force en translation Z (N)
        self.force_rx = 0.0  # Couple en rotation X (Nm)
        self.force_ry = 0.0  # Couple en rotation Y (Nm)
        self.force_rz = 0.0  # Couple en rotation Z (Nm)

        # ============================================================================
        # VARIABLES POUR LE CALCUL DE VITESSE ET D'ACCÉLÉRATION
        # ============================================================================
        
        # Positions précédentes pour le calcul de vitesse
        self.distance_x_prev = 0.0
        self.distance_y_prev = 0.0
        self.distance_z_prev = 0.0
        
        # Accélérations calculées (m/s² ou rad/s²)
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0

        # ============================================================================
        # PARAMÈTRES TEMPORELS ET DE SÉCURITÉ
        # ============================================================================
        
        # Timeout pour considérer les données du contrôleur comme obsolètes (en secondes)
        self.controller_data_timeout = 0.5

        # Paramètres physiques du système
        self.masse = 1.0      # Masse du robot (kg)
        self.masse_obj = 0.0  # Masse de l'objet manipulé (kg)

        # Variables pour le calcul de vitesse en temps réel
        self.previous_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]
        self.current_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # Vitesses correspondantes
        self.previous_time = time.time()
        self.controller_data_initialized = False

        # ============================================================================
        # PARAMÈTRES DE CONTRÔLE
        # ============================================================================
        
        # Coefficients d'amortissement pour stabiliser le mouvement
        self.damping_coeff_translation = 2.0  # Amortissement en translation (N·s/m)
        self.damping_coeff_rotation = 0.05    # Amortissement en rotation (Nm·s/rad)
        
        # Variables pour le calcul du dt (delta temps) réel
        self.last_time = time.time()
        self.last_controller_update_time = time.time()
        
        # Flags de contrôle d'état
        self.controller_pose_received = False  # Indique si la position du contrôleur a été reçue
        self.in_speed_mode = False            # Par défaut, on est en mode position
        
        # ============================================================================
        # DÉFINITION DES LIMITES DE SÉCURITÉ
        # ============================================================================
        
        # Limites pour le retour de force par axe
        # Structure: [seuil_inférieur, seuil_supérieur, limite_max, limite_min]
        # Ces valeurs définissent les zones de sécurité pour chaque axe
        self.value_for_deplacement = [
            [0.22, 0.28, 0.37, 0.13],           # Axe X translation (m)
            [-0.05, 0.01, 0.23, -0.27],         # Axe Y translation (m)
            [0.01, 0.07, 0.19, -0.1],           # Axe Z translation (m)
            [-0.2, 0.2, 1.6, -1.6],             # Rotation autour de X (Roll) (rad)
            [1.0, 1.3, 1.4, 0.6],               # Rotation autour de Y (Pitch) (rad)
            [-0.3, 0.3, 2, -2],                 # Rotation autour de Z (Yaw) (rad)
        ]
        
        # ============================================================================
        # CONSTANTES DE FORCE
        # ============================================================================
        
        # Forces maximales par axe
        self.max_force_by_axis = [3.0, 3.0, 3.0]      # Force maximale en translation (N)
        self.max_torque_by_axis = [0.15, 0.2, 0.075]  # Couple maximum en rotation (Nm)
        self.stiffness_rotation = 4.0                  # Coefficient de raideur en rotation
        
        # ============================================================================
        # VARIABLES DE DEBUG ET TIMING
        # ============================================================================
        
        # Compteur pour l'affichage périodique des messages de debug
        self.count = 0
        
        # Création d'un timer pour l'envoi périodique des forces
        self.timer = self.create_timer(0.1, self.send_force)  # Appel à 10Hz
        
        self.get_logger().info('Nœud de création de force joystick démarré')

    def mode_callback(self, msg):
        """
        Callback pour recevoir et traiter le mode de contrôle actuel.
        
        Le mode détermine si le robot est contrôlé en position ou en vitesse.
        En mode position, aucune force n'est appliquée. En mode vitesse,
        les forces de retour sont calculées et appliquées.
        
        Args:
            msg (Float32MultiArray): Message contenant le mode de contrôle
                Format attendu: [mode_position, mode_vitesse, x, y]
                - mode_position = 1.0 si en mode position, 0.0 sinon
                - mode_vitesse = 1.0 si en mode vitesse, 0.0 sinon
                
        Note:
            Les modes sont exclusifs (un seul peut être actif à la fois)
        """
        # Vérification de la validité du message
        if len(msg.data) < 4:
            self.get_logger().error(f"Message Mode_Pose_Vitesse invalide: attendu au moins 4 valeurs, reçu {len(msg.data)}")
            return
            
        # Extraction du mode vitesse (index 1)
        # Mode Position = [1.0, 0.0, x, y], Mode Vitesse = [0.0, 1.0, x, y]
        self.in_speed_mode = bool(msg.data[1])  # 1.0 si en mode vitesse, 0.0 sinon
        
        self.get_logger().debug(f"Mode mis à jour: {'Vitesse' if self.in_speed_mode else 'Position'}")

    def distance_obj_callback(self, msg):
        """
        Callback pour recevoir la distance entre le robot et les objets proches.
        
        Cette information est utilisée pour calculer des forces de répulsion
        et déterminer si un objet est saisi par la pince.
        
        Args:
            msg (Float32MultiArray): Message contenant la distance aux objets
                Format attendu: [distance, param2, param3, param4]
                - distance: Distance minimale aux objets proches (m)
                
        Note:
            Si la distance est très faible (<0.1m) et que la pince est fermée,
            on considère qu'un objet est saisi (ajout de masse virtuelle).
        """
        # Vérification de la validité du message
        if len(msg.data) < 4:
            self.get_logger().error(f"Message distance_robot_objet invalide: attendu au moins 4 valeurs, reçu {len(msg.data)}")
            return

        # Extraction de la distance aux objets
        self.distance_objet = msg.data[0]
        
        # Calcul de la force de répulsion basée sur la distance
        self.force_distance_objet = 0.0  # Formule pour augmenter le retour de force quand on s'approche de l'objet: (1/self.distance_objet) / 5

        self.get_logger().debug(f"Distance entre le robot et l'objet : {self.distance_objet}")
        self.get_logger().debug(f"Force en fonction de la distance entre le robot et l'objet : {self.force_distance_objet}")

        # Détection de la saisie d'un objet
        if self.distance_objet < 0.1 and self.pince <= 0.02:
            self.masse_obj = 0.2  # Ajout de masse virtuelle pour l'objet saisi
        
    def position_controleur_sub_callback(self, msg):
        """
        Callback principal pour recevoir la position et l'orientation du contrôleur haptique.
        
        Cette fonction traite les données du contrôleur, applique les transformations
        nécessaires, calcule les vitesses et met à jour l'état du système.
        
        Args:
            msg (Float32MultiArray): Message contenant la pose du contrôleur
                Format attendu: [x, y, z, qx, qy, qz, qw, ...]
                - x, y, z: Position cartésienne (m)
                - qx, qy, qz, qw: Orientation en quaternion
                
        Note:
            Une rotation de 22.5° autour de l'axe Y est appliquée pour
            corriger l'orientation du contrôleur par rapport au robot.
        """
        # Vérification de la validité du message
        if len(msg.data) < 8:
            self.get_logger().error(f"Message valeur_effecteur invalide: attendu au moins 8 valeurs, reçu {len(msg.data)}")
            return
        
        # Extraction des données de position cartésienne
        self.controleur_x = float(msg.data[0])
        self.controleur_y = float(msg.data[1])
        self.controleur_z = float(msg.data[2])
        
        # Extraction des données d'orientation (quaternion)
        self.controleur_qx = float(msg.data[3])
        self.controleur_qy = float(msg.data[4])
        self.controleur_qz = float(msg.data[5])
        self.controleur_qw = float(msg.data[6])

        # ============================================================================
        # CORRECTION D'ORIENTATION DU CONTRÔLEUR
        # ============================================================================
        
        # Application d'une rotation de 22.5° autour de l'axe Y
        # Cette correction compense la différence d'orientation entre
        # le référentiel de la base contrôleur et celui de l'effecteur
        angle = math.pi / 8  # 22.5° en radians
        q_rot = (math.cos(angle/2), 0, math.sin(angle/2), 0)  # Quaternion de rotation

        # Multiplication des quaternions pour appliquer la rotation
        q_orig = (self.controleur_qw, self.controleur_qx, self.controleur_qy, self.controleur_qz)
        self.controleur_qw, self.controleur_qx, self.controleur_qy, self.controleur_qz = self.quaternion_multiply(q_orig, q_rot)
        
        # ============================================================================
        # CONVERSION EN ANGLES D'EULER
        # ============================================================================
        
        # Conversion du quaternion en angles d'Euler pour faciliter les calculs
        self.euler_x, self.euler_y, self.euler_z = self.quaternion_to_euler(
            self.controleur_qx, self.controleur_qy, self.controleur_qz, self.controleur_qw
        )
        
        # ============================================================================
        # CALCUL DE LA VITESSE
        # ============================================================================
        
        # Calcul de la vitesse basé sur les positions précédentes
        self.calculate_velocity()
        
        # ============================================================================
        # MISE À JOUR DES FLAGS D'ÉTAT
        # ============================================================================
        
        # Marquer que les données du contrôleur ont été reçues
        self.controller_pose_received = True
        self.controller_data_initialized = True
        self.last_controller_update_time = time.time()

        # Messages de debug périodiques
        self.get_logger().debug(f"Position contrôleur reçue: [{self.controleur_x}, {self.controleur_y}, {self.controleur_z}] " +
                               f"[{self.controleur_qx}, {self.controleur_qy}, {self.controleur_qz}, {self.controleur_qw}]")
        self.get_logger().debug(f"Angles d'Euler correspondants: [Roll={math.degrees(self.euler_x):.2f}°, " +
                               f"Pitch={math.degrees(self.euler_y):.2f}°, Yaw={math.degrees(self.euler_z):.2f}°]")

    def joint_state_callback(self, msg):
        """
        Callback pour recevoir l'état actuel des articulations du robot.
        
        Cette fonction met à jour l'état des articulations et surveille
        particulièrement l'état de la pince pour détecter la saisie d'objets.
        
        Args:
            msg (JointState): Message contenant l'état des articulations
                - name: Liste des noms des articulations
                - position: Liste des positions correspondantes (rad ou m)
                - velocity: Liste des vitesses (optionnel)
                - effort: Liste des efforts (optionnel)
        """
        # Mise à jour de l'état de toutes les articulations
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
                
                # Surveillance spéciale de la pince
                if 'fr3_finger_joint1' in name.lower() or 'fr3_finger_joint2' in name.lower():
                    self.pince = abs(msg.position[i])  # Valeur absolue de l'ouverture de la pince

    def calculate_velocity(self):
        """
        Calcule la vitesse du contrôleur basée sur les positions actuelles et précédentes.
        
        Cette fonction utilise la dérivée numérique pour calculer les vitesses
        linéaires et angulaires en temps réel. Les vitesses sont utilisées pour
        l'amortissement et la stabilisation du contrôle.
        
        Returns:
            None: Les vitesses sont stockées dans self.current_velocity
            
        Note:
            Le calcul est effectué pour les 6 degrés de liberté :
            - 3 translations (x, y, z)
            - 3 rotations (roll, pitch, yaw)
        """
        current_time = time.time()
        
        # Calcul du delta temps
        dt = current_time - self.previous_time
        
        # Protection contre les intervalles de temps trop petits
        if dt < 0.001:  # Éviter la division par zéro
            return
            
        # Initialisation lors du premier appel
        if not self.controller_data_initialized:
            self.previous_position = [
                self.controleur_x, self.controleur_y, self.controleur_z,
                self.euler_x, self.euler_y, self.euler_z
            ]
            self.previous_time = current_time
            return
        
        # Construction du vecteur de position actuel
        current_cartesian = [
            self.controleur_x, self.controleur_y, self.controleur_z,
            self.euler_x, self.euler_y, self.euler_z
        ]
        
        # Calcul des vitesses pour tous les axes par dérivée numérique
        for i in range(6):
            self.current_velocity[i] = (current_cartesian[i] - self.previous_position[i]) / dt
        
        # Sauvegarde des positions pour le prochain calcul
        self.previous_position = current_cartesian.copy()
        self.previous_time = current_time
        
        # Affichage périodique des vitesses pour debug
        if self.count % 100 == 0:
            self.get_logger().debug(f"Vitesses - Trans: [{self.current_velocity[0]:.3f}, {self.current_velocity[1]:.3f}, {self.current_velocity[2]:.3f}] " +
                                  f"Rot: [{math.degrees(self.current_velocity[3]):.1f}°/s, {math.degrees(self.current_velocity[4]):.1f}°/s, {math.degrees(self.current_velocity[5]):.1f}°/s]")

    def send_force(self):
        """
        Fonction principale de calcul et d'envoi des forces de retour haptique.
        
        Cette fonction est appelée périodiquement (10Hz) et calcule les forces
        à appliquer au contrôleur haptique basées sur :
        - La position actuelle du robot par rapport aux limites de sécurité
        - La vitesse du contrôleur (pour l'amortissement)
        - L'accélération du contrôleur (pour la compensation d'inertie)
        - La distance aux objets proches
        
        Les forces sont envoyées uniquement en mode vitesse. En mode position,
        des forces nulles sont envoyées.
        
        Note:
            Le calcul comprend deux composantes principales :
            1. Force de rappel (proportionnelle à la distance des limites)
            2. Force d'amortissement (proportionnelle à la vitesse)
        """
        # Vérification que les données du contrôleur sont disponibles
        if not self.controller_pose_received:
            return
            
        # Incrémenter le compteur pour affichage périodique
        self.count += 1
        
        # Vecteur pour stocker les forces calculées
        total_force = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [tx, ty, tz, rx, ry, rz]
        
        # ============================================================================
        # GESTION DU MODE POSITION
        # ============================================================================
        
        # En mode position, aucune force n'est appliquée
        if not self.in_speed_mode:
            # Création et envoi du message avec des forces nulles
            force_message = Float32MultiArray()
            force_message.data = total_force
            self.force_joystick_pub.publish(force_message)
            
            # Affichage périodique du statut
            if self.count % 1000 == 0:
                self.get_logger().debug("Mode Position: Aucune force appliquée")
            
            return
        
        # ============================================================================
        # GESTION DU MODE VITESSE - VÉRIFICATIONS DE SÉCURITÉ
        # ============================================================================
        
        # Indicateurs pour les zones critiques
        critical_axis = False
        warning_axis = False
        
        # Vérifier si les données du contrôleur sont trop anciennes
        current_time = time.time()
        if not self.controller_data_initialized or (current_time - self.last_controller_update_time) > self.controller_data_timeout:
            # Données obsolètes, ne pas calculer de forces
            force_message = Float32MultiArray()
            force_message.data = total_force
            self.force_joystick_pub.publish(force_message)
            
            if self.count % 100 == 0:
                self.get_logger().debug("Données du contrôleur obsolètes - Forces mises à zéro")
            return
        
        # ============================================================================
        # PRÉPARATION DES DONNÉES POUR LE CALCUL
        # ============================================================================
        
        # Construction du vecteur de positions cartésiennes
        cartesian_values = [
            self.controleur_x, self.controleur_y, self.controleur_z,
            self.euler_x, self.euler_y, self.euler_z
        ]

        # Calcul de l'accélération à chaque cycle
        self.calcul_accel()

        # ============================================================================
        # CALCUL DES FORCES POUR CHAQUE AXE
        # ============================================================================
        
        # Parcours des 6 axes (3 translations + 3 rotations)
        for axis in range(6):
            # Récupération de la position actuelle sur cet axe
            axis_position = cartesian_values[axis]
            
            # Récupération des limites de sécurité pour cet axe
            limits = self.value_for_deplacement[axis]
            lower_threshold, upper_threshold, max_limit, min_limit = limits
            
            # --------------------------------------------------------------------
            # CALCUL DE LA FORCE DE RAPPEL (REPULSION)
            # --------------------------------------------------------------------
            
            repulsion_force = 0.0
            in_warning_zone = False
            in_critical_zone = False
            
            # Vérification si la position dépasse les seuils de sécurité
            if axis_position < lower_threshold:
                # Position en dessous du seuil inférieur
                distance = lower_threshold - axis_position
                range_val = lower_threshold - min_limit
                
                if range_val > 0:
                    # Normalisation de la distance (0 à 1)
                    normalized_distance = distance / range_val
                    
                    # Calcul de la force en fonction du type d'axe
                    if axis < 3:  # Translation
                        accel = [self.accel_x, self.accel_y, self.accel_z][axis]
                        # Force = raideur + compensation d'inertie + répulsion d'objet + offset
                        repulsion_force = normalized_distance * self.max_force_by_axis[axis] + (self.masse + self.masse_obj) * accel + self.force_distance_objet + 1
                    else:  # Rotation
                        # Force rotationnelle avec coefficient de raideur
                        repulsion_force = self.stiffness_rotation * normalized_distance * self.max_torque_by_axis[axis-3] / 5
                    
                    # Déterminer les zones d'avertissement et critiques
                    if normalized_distance > 0.25:
                        in_warning_zone = True
                    if normalized_distance > 0.6:
                        in_critical_zone = True
            
            elif axis_position > upper_threshold:
                # Position au dessus du seuil supérieur
                distance = axis_position - upper_threshold
                range_val = max_limit - upper_threshold
                
                if range_val > 0:
                    # Normalisation de la distance (0 à 1)
                    normalized_distance = distance / range_val
                    
                    # Calcul de la force en fonction du type d'axe (force négative)
                    if axis < 3:  # Translation
                        accel = [self.accel_x, self.accel_y, self.accel_z][axis]
                        # Force opposée pour ramener vers la zone de sécurité
                        repulsion_force = - (normalized_distance * self.max_force_by_axis[axis] + (self.masse + self.masse_obj) * accel + self.force_distance_objet + 1)
                    else:  # Rotation
                        # Force rotationnelle opposée
                        repulsion_force = - (self.stiffness_rotation * normalized_distance * self.max_torque_by_axis[axis-3]) / 5
                    
                    # Déterminer les zones d'avertissement et critiques
                    if normalized_distance > 0.25:
                        in_warning_zone = True
                    if normalized_distance > 0.6:
                        in_critical_zone = True
            
            # --------------------------------------------------------------------
            # CALCUL DE LA FORCE D'AMORTISSEMENT
            # --------------------------------------------------------------------

            # La force d'amortissement s'oppose à la vitesse pour stabiliser le mouvement
            # Elle est proportionnelle à la vitesse actuelle et utilise des coefficients
            # différents selon le type d'axe (translation vs rotation)
            damping_force = 0.0

            if axis < 3:  # Translation
                damping_force = -self.damping_coeff_translation * self.current_velocity[axis]
            elif 3 < axis < 6:  # Rotation
                damping_force = -self.damping_coeff_rotation * self.current_velocity[axis] / 5
            else :
                damping_force = -self.damping_coeff_rotation * self.current_velocity[axis] / 50
            
            # FORCE TOTALE: Combinaison de la force de rappel et de l'amortissement
            # La force de rappel provient du calcul précédent dans la fonction parente
            total_axis_force = repulsion_force + damping_force

            # --------------------------------------------------------------------
            # LIMITATION DES FORCES
            # --------------------------------------------------------------------
            
            # Déterminer la force maximale autorisée selon le type d'axe
            if axis < 3:  # Translation
                max_force = self.max_force_by_axis[axis]
            else:  # Rotation
                max_force = self.max_torque_by_axis[axis-3]
            
            # Appliquer les limites pour éviter les forces excessives
            # Clamping entre -max_force et +max_force
            total_axis_force = max(-max_force, min(max_force, total_axis_force))
            
            # Assigner la force calculée au vecteur de forces totales
            total_force[axis] = total_axis_force
            
            # --------------------------------------------------------------------
            # MISE À JOUR DES DRAPEAUX D'ÉTAT
            # --------------------------------------------------------------------

            # Mettre à jour les drapeaux globaux pour l'interface utilisateur
            if in_critical_zone:
                critical_axis = True
            if in_warning_zone:
                warning_axis = True
            
            # --------------------------------------------------------------------
            # LOGGING ET DEBUG
            # --------------------------------------------------------------------

            # Log debug info every 100 cycles pour éviter de surcharger les logs
            if self.count % 100 == 0:
                # Noms des axes pour l'affichage
                axis_name = ["X trans", "Y trans", "Z trans", "Roll", "Pitch", "Yaw"][axis]
                debug_str = f"Axe {axis_name}: "
                
                # Formatage différent selon le type d'axe
                if axis < 3:  # Translation - valeurs en mètres et Newtons
                    debug_str += f"Pos={axis_position:.3f}m Rappel={repulsion_force:.3f}N Amort={damping_force:.3f}N Total={total_axis_force:.3f}N"
                else:  # Rotation - valeurs en degrés et Newton-mètres
                    debug_str += f"Pos={math.degrees(axis_position):.1f}° Rappel={repulsion_force:.3f}Nm Amort={damping_force:.3f}Nm Total={total_axis_force:.3f}Nm"
                
                # Ajout d'indicateurs de zone critique ou d'attention
                if in_critical_zone:
                    debug_str += " [CRITIQUE]"
                elif in_warning_zone:
                    debug_str += " [ATTENTION]"
                
                self.get_logger().debug(debug_str)
        
        # --------------------------------------------------------------------
        # PUBLICATION DES FORCES
        # --------------------------------------------------------------------

        # Créer le message ROS2 pour publier les forces calculées
        force_message = Float32MultiArray()

        # Remplir les données de force (6 axes: X, Y, Z, Roll, Pitch, Yaw)
        force_message.data = total_force

        # Publier le message de force vers le contrôleur haptique
        self.force_joystick_pub.publish(force_message)

        # Affichage périodique des forces totales pour monitoring
        if self.count % 100 == 0:
            self.get_logger().debug(f"Mode Vitesse - Forces totales: F[{total_force[0]:.2f}, {total_force[1]:.2f}, {total_force[2]:.2f}] " +
                                f"T[{total_force[3]:.2f}, {total_force[4]:.2f}, {total_force[5]:.2f}]")
            
            # Affichage du statut des zones d'attention et critiques
            if critical_axis:
                self.get_logger().debug("ATTENTION: Axes en zone CRITIQUE")
            elif warning_axis:
                self.get_logger().debug("Axes en zone d'ATTENTION")

    def quaternion_multiply(self, q1, q2):
        """
        Multiplie deux quaternions selon la formule de multiplication des quaternions.
        
        La multiplication de quaternions est utilisée pour composer des rotations.
        Si q1 représente une rotation R1 et q2 une rotation R2, alors q1*q2 
        représente la rotation composée R2 suivie de R1.
        
        Args:
            q1 (tuple): Premier quaternion au format (w, x, y, z)
                    où w est la partie scalaire et (x, y, z) la partie vectorielle
            q2 (tuple): Deuxième quaternion au format (w, x, y, z)
        
        Returns:
            tuple: Quaternion résultant de la multiplication au format (w, x, y, z)
        
        Note:
            La multiplication des quaternions n'est pas commutative: q1*q2 ≠ q2*q1
            
        Exemple:
            >>> q1 = (1, 0, 0, 0)  # Quaternion identité
            >>> q2 = (0.707, 0, 0, 0.707)  # Rotation de 90° autour de Z
            >>> result = self.quaternion_multiply(q1, q2)
            >>> # result représente la même rotation que q2
        """
        # Extraction des composantes des quaternions
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        # Calcul selon la formule de multiplication des quaternions
        # q1 * q2 = (w1*w2 - x1*x2 - y1*y2 - z1*z2) + 
        #           (w1*x2 + x1*w2 + y1*z2 - z1*y2)i +
        #           (w1*y2 - x1*z2 + y1*w2 + z1*x2)j +
        #           (w1*z2 + x1*y2 - y1*x2 + z1*w2)k
        return (
            w1*w2 - x1*x2 - y1*y2 - z1*z2,  # Partie scalaire (w)
            w1*x2 + x1*w2 + y1*z2 - z1*y2,  # Partie vectorielle (x)
            w1*y2 - x1*z2 + y1*w2 + z1*x2,  # Partie vectorielle (y)
            w1*z2 + x1*y2 - y1*x2 + z1*w2   # Partie vectorielle (z)
        )

    def quaternion_to_euler(self, qx, qy, qz, qw):
        """
        Convertit un quaternion en angles d'Euler selon la convention ZYX (yaw-pitch-roll).
        
        Cette conversion est essentielle pour le contrôle haptique car elle permet
        de transformer l'orientation du contrôleur (représentée par un quaternion)
        en angles d'Euler plus intuitifs pour le calcul des forces de retour.
        
        Args:
            qx (float): Composante x du quaternion
            qy (float): Composante y du quaternion  
            qz (float): Composante z du quaternion
            qw (float): Composante w (scalaire) du quaternion
        
        Returns:
            tuple: Angles d'Euler (roll, pitch, yaw) en radians
                - roll: Rotation autour de l'axe X (roulis)
                - pitch: Rotation autour de l'axe Y (tangage)  
                - yaw: Rotation autour de l'axe Z (lacet)
        
        Note:
            - La convention ZYX signifie que les rotations sont appliquées dans l'ordre:
            1. Yaw (rotation autour de Z)
            2. Pitch (rotation autour de Y)
            3. Roll (rotation autour de X)
            - Gestion du gimbal lock pour pitch > 1.2 radians (~69°)
            - Les angles sont normalisés dans la plage [-π, π]
        
        Raises:
            Aucune exception, mais gère le gimbal lock en remettant roll et yaw à 0
        """
        # Normalisation du quaternion pour éviter les erreurs numériques
        # Un quaternion doit avoir une norme unitaire pour représenter une rotation pure
        norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        qw /= norm
        qx /= norm
        qy /= norm
        qz /= norm
        
        # --------------------------------------------------------------------
        # CALCUL DU ROLL (rotation autour de X)
        # --------------------------------------------------------------------
        # Utilisation de la formule: roll = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx² + qy²))
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # --------------------------------------------------------------------
        # CALCUL DU PITCH (rotation autour de Y)
        # --------------------------------------------------------------------
        # Utilisation de la formule: pitch = asin(2*(qw*qy - qz*qx))
        sinp = 2 * (qw * qy - qz * qx)
        
        # Gestion du cas où sinp est hors de la plage [-1, 1] (erreurs numériques)
        if abs(sinp) >= 1:
            # Limitation à ±90° pour éviter les erreurs de calcul
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # --------------------------------------------------------------------
        # CALCUL DU YAW (rotation autour de Z)
        # --------------------------------------------------------------------
        # Utilisation de la formule: yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # --------------------------------------------------------------------
        # GESTION DU GIMBAL LOCK
        # --------------------------------------------------------------------
        # Le gimbal lock se produit quand le pitch approche ±90°
        # Dans ce cas, roll et yaw deviennent indéterminés
        if abs(pitch) > 1.2:  # ~69° - seuil de sécurité avant gimbal lock complet
            # Remise à zéro des angles problématiques
            roll = 0.0
            yaw = 0.0
        
        return roll, pitch, yaw

    def calcul_accel(self):
        """
        Calcule l'accélération du contrôleur haptique basée sur les positions actuelles et précédentes.
        
        Cette fonction est cruciale pour le retour haptique car elle détermine la sensation
        de "réactivité" du contrôleur. L'accélération est calculée par différenciation
        numérique des positions et est utilisée pour générer des forces proportionnelles
        au mouvement du contrôleur.
        
        Returns:
            tuple: Accélérations (accel_x, accel_y, accel_z) en m/s²
        
        Note:
            - Appelée systématiquement dans send_force() pour garantir un calcul continu
            - Gère les timeouts des données du contrôleur pour éviter les comportements erratiques
            - Applique des limites d'accélération pour éviter les forces excessives
            - Utilise un filtrage temporel pour éviter les dt trop petits ou grands
        
        Algorithme:
            1. Vérification de la validité des données du contrôleur
            2. Calcul du delta temps depuis le dernier calcul
            3. Calcul des déplacements en position
            4. Calcul de l'accélération par différenciation numérique
            5. Application des limites de sécurité
            6. Mise à jour des positions précédentes
        """
        current_time = time.time()
        
        # --------------------------------------------------------------------
        # VÉRIFICATION DE LA VALIDITÉ DES DONNÉES
        # --------------------------------------------------------------------
        # Vérifier si les données du contrôleur sont trop anciennes ou inexistantes
        # Ceci évite les comportements erratiques en cas de perte de communication
        if not self.controller_pose_received or (current_time - self.last_controller_update_time) > self.controller_data_timeout:
            # Données obsolètes ou inexistantes, sécurité: accélération nulle
            self.accel_x = 0.0
            self.accel_y = 0.0
            self.accel_z = 0.0
            
            # Log périodique pour debugging
            if self.count % 100 == 0:
                self.get_logger().debug("Données du contrôleur obsolètes ou manquantes - Accélération mise à zéro")
            return self.accel_x, self.accel_y, self.accel_z
        
        # --------------------------------------------------------------------
        # CALCUL DU DELTA TEMPS
        # --------------------------------------------------------------------
        # Calculer le temps écoulé depuis le dernier calcul d'accélération
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Filtrage du dt pour éviter les problèmes numériques
        if dt < 0.001:  # Minimum 1ms - évite division par zéro et oscillations
            dt = 0.001
        elif dt > 1.0:  # Maximum 1s - évite les sauts après une pause
            dt = 1.0
        
        # --------------------------------------------------------------------
        # CALCUL DES DÉPLACEMENTS
        # --------------------------------------------------------------------
        # Calculer les déplacements (différence entre position actuelle et précédente)
        delta_x = self.controleur_x - self.distance_x_prev
        delta_y = self.controleur_y - self.distance_y_prev
        delta_z = self.controleur_z - self.distance_z_prev

        # --------------------------------------------------------------------
        # CALCUL DE L'ACCÉLÉRATION
        # --------------------------------------------------------------------
        # Calculer les accélérations seulement si le déplacement est significatif
        # Seuil de 0.001m (1mm) pour éviter le bruit de mesure
        if abs(delta_x) > 0.001 or abs(delta_y) > 0.001 or abs(delta_z) > 0.001:
            # Accélération = déplacement / temps²
            # Note: Cette formule est une approximation de la dérivée seconde
            self.accel_x = delta_x / (dt**2)
            self.accel_y = delta_y / (dt**2)
            self.accel_z = delta_z / (dt**2)
        else:
            # Contrôleur stationnaire ou presque - accélération nulle
            self.accel_x = 0.0
            self.accel_y = 0.0
            self.accel_z = 0.0

        # --------------------------------------------------------------------
        # LIMITATION DES ACCÉLÉRATIONS
        # --------------------------------------------------------------------
        # Appliquer des limites de sécurité pour éviter les forces excessives
        # Limite à ±2 m/s² pour un comportement haptique réaliste
        
        # Limitation avec inversion pour correction automatique
        if self.accel_x > 2:
            self.accel_x = -2.0  # Inversion pour effet correcteur
        elif self.accel_x < -2:
            self.accel_x = 2.0

        if self.accel_y > 2:
            self.accel_y = -2.0
        elif self.accel_y < -2:
            self.accel_y = 2.0

        if self.accel_z > 2:
            self.accel_z = -2.0
        elif self.accel_z < -2:
            self.accel_z = 2.0
        
        # --------------------------------------------------------------------
        # MISE À JOUR DES POSITIONS PRÉCÉDENTES
        # --------------------------------------------------------------------
        # Sauvegarder les positions actuelles pour le prochain calcul
        self.distance_x_prev = self.controleur_x
        self.distance_y_prev = self.controleur_y
        self.distance_z_prev = self.controleur_z

        # --------------------------------------------------------------------
        # LOGGING DE DEBUG
        # --------------------------------------------------------------------
        # Afficher l'information d'accélération périodiquement pour monitoring
        if self.count % 100 == 0:
            self.get_logger().debug(f"dt: {dt:.3f}s - Accélération suivant x: {self.accel_x:.2f}  y: {self.accel_y:.2f}  z: {self.accel_z:.2f}")
        
        return self.accel_x, self.accel_y, self.accel_z

def main(args=None):
    """
    Point d'entrée principal du nœud ROS2 de contrôle haptique.
    
    Cette fonction initialise le système ROS2, crée une instance du nœud Joystick
    (qui gère l'interface haptique), et maintient le nœud actif jusqu'à arrêt.
    
    Args:
        args (list, optional): Arguments de ligne de commande passés à ROS2.
                              Par défaut None, utilise sys.argv automatiquement.
    
    Workflow:
        1. Initialisation du système ROS2
        2. Création du nœud Joystick (contrôleur haptique)
        3. Démarrage de la boucle d'événements ROS2 (spin)
        4. Nettoyage et arrêt propre du système
    
    Note:
        - La fonction spin() bloque jusqu'à ce que le nœud soit arrêté (Ctrl+C)
        - Le nettoyage automatique garantit la libération des ressources
        - Compatible avec le système de lancement ROS2 (ros2 run/launch)
    """
    # Initialisation du système ROS2 avec les arguments fournis
    rclpy.init(args=args)
    
    # Création de l'instance du nœud de contrôle haptique
    # Le nœud Joystick gère toute l'interface avec le contrôleur Desktop 6D
    node = Joystick()
    
    # Démarrage de la boucle d'événements ROS2
    # Cette fonction bloque et traite les messages/services jusqu'à arrêt
    rclpy.spin(node)
    
    # Nettoyage et destruction du nœud
    # Libère les ressources et ferme les connexions proprement
    node.destroy_node()
    
    # Arrêt final du système ROS2
    rclpy.shutdown()