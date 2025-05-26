#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
import math
import time

class Joystick(Node):
    def __init__(self):
        super().__init__('joystick')

        # Création d'un publisher pour la force du contrôleur
        self.force_joystick_pub = self.create_publisher(
            Float32MultiArray,
            '/force_joystick',
            10
        )
        
        # Création du subscriber pour la pose du contrôleur
        self.position_controleur_sub = self.create_subscription(
            Float32MultiArray, 
            '/valeur_effecteur', 
            self.position_controleur_sub_callback, 
            10
        )

        # Création du subscriber pour la pose de l'effecteur du robot
        self.position_effecteur_robot_sub = self.create_subscription(
            TFMessage, 
            '/position_effecteur_robot', 
            self.position_effecteur_robot_callback, 
            10
        )
        
        # Création du subscriber pour le mode (position/vitesse)
        self.mode_sub = self.create_subscription(
            Float32MultiArray,
            '/Mode_Pose_Vitesse',
            self.mode_callback,
            10
        )

        # Créer un subscriber pour l'état actuel des articulations
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Création du subscriber pour la position de l'objet
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/fiole/pose',
            self.pose_callback,
            10
        )

        # Création du subscriber pour la distance entre parroies et robot
        self.distance_parroies_sub = self.create_subscription(
            Float32MultiArray,
            '/distance_robot_parroies',
            self.distance_parroies_callback,
            10
        )

        # Initialiser la position du contrôleur
        self.controleur_x = 0.0
        self.controleur_y = 0.0
        self.controleur_z = 0.0
        self.controleur_qx = 0.0
        self.controleur_qy = 0.0
        self.controleur_qz = 0.0
        self.controleur_qw = 1.0  # Quaternion d'identité (pas de rotation)

        # Initialiser la position de l'objet
        self.pose_objet_x = 0.0
        self.pose_objet_y = 0.0
        self.pose_objet_z = 0.0
        self.pose_objet_qx = 0.0
        self.pose_objet_qy = 0.0
        self.pose_objet_qz = 0.0
        self.pose_objet_qw = 0.0

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
        
        # Pour stocker les angles d'Euler correspondants
        self.euler_x = 0.0  # Roll
        self.euler_y = 0.0  # Pitch
        self.euler_z = 0.0  # Yaw

        # Initialiser la force du contrôleur
        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        self.force_rx = 0.0
        self.force_ry = 0.0
        self.force_rz = 0.0

        # Initialiser les distances précédentes
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
        self.controller_pose_received = False
        self.in_speed_mode = False  # Par défaut, on est en mode position
        self.scale_factor = 1.0     # Facteur d'échelle
        
        # Définir les limites pour le retour de force
        # Structure: [seuil_inférieur, seuil_supérieur, limite_max, limite_min]
        self.value_for_deplacement = [
            [0.22, 0.28, 0.37, 0.13],           # Axe X translation
            [-0.05, 0.01, 0.23, -0.27],         # Axe Y translation
            [0.01, 0.07, 0.19, -0.1],           # Axe Z translation
            [-0.2, 0.2, 0.8, -0.8],             # Rotation autour de l'axe x (Roll) en radians
            [0.4, 0.8, 1.5, -0.3],              # Rotation autour de l'axe y (Pitch) en radians
            [-0.225, 0.225, 0.9, -0.9],         # Rotation autour de l'axe z (Yaw) en radians
        ]
        
        # Définir les constantes de force
        self.max_force_by_axis = [3.0, 3.0, 3.0]      # Force maximale en translation (N)
        self.max_torque_by_axis = [0.1, 0.1, 0.1]     # Couple maximum en rotation (Nm)
        self.stiffness_rotation = 4.0               # Coefficient de raideur en rotation
        
        # Compteur pour les messages de debug
        self.count = 0
        
        # Créer un timer pour appeler périodiquement send_force
        self.timer = self.create_timer(0.1, self.send_force)  # Appel à 10Hz
        
        self.get_logger().info('Nœud de création de force joystick démarré')

    # Callback pour recevoir le mode (position/vitesse)
    def mode_callback(self, msg):
        """Callback pour recevoir le mode actuel"""
        if len(msg.data) < 4:
            self.get_logger().error(f"Message Mode_Pose_Vitesse invalide: attendu au moins 4 valeurs, reçu {len(msg.data)}")
            return
            
        # Mode Position = [1.0, 0.0, x, y], Mode Vitesse = [0.0, 1.0, x, y]
        self.in_speed_mode = bool(msg.data[1])  # 1.0 si en mode vitesse
        self.scale_factor = float(msg.data[3])
        
        self.get_logger().debug(f"Mode mis à jour: {'Vitesse' if self.in_speed_mode else 'Position'}, Scale: {self.scale_factor}")
        
    # Callback pour recevoir la position du contrôleur
    def position_controleur_sub_callback(self, msg):
        """Callback pour recevoir la position du contrôleur"""

        # Vérifier que le message contient assez de données
        if len(msg.data) < 8:
            self.get_logger().error(f"Message valeur_effecteur invalide: attendu au moins 8 valeurs, reçu {len(msg.data)}")
            return
        
        # Mettre à jour la position du contrôleur
        self.controleur_x = float(msg.data[0])
        self.controleur_y = float(msg.data[1])
        self.controleur_z = float(msg.data[2])
        self.controleur_qx = float(msg.data[3])
        self.controleur_qy = float(msg.data[4])
        self.controleur_qz = float(msg.data[5])
        self.controleur_qw = float(msg.data[6])
        
        # Convertir le quaternion en angles d'Euler
        self.euler_x, self.euler_y, self.euler_z = self.quaternion_to_euler(
            self.controleur_qx, self.controleur_qy, self.controleur_qz, self.controleur_qw
        )
        
        self.controller_pose_received = True
        self.last_controller_update_time = time.time()

        self.get_logger().debug(f"Position contrôleur reçue: [{self.controleur_x}, {self.controleur_y}, {self.controleur_z}] " +
                               f"[{self.controleur_qx}, {self.controleur_qy}, {self.controleur_qz}, {self.controleur_qw}]")
        self.get_logger().debug(f"Angles d'Euler correspondants: [Roll={math.degrees(self.euler_x):.2f}°, " +
                               f"Pitch={math.degrees(self.euler_y):.2f}°, Yaw={math.degrees(self.euler_z):.2f}°]")

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

    def joint_state_callback(self, msg):
        """Callback pour recevoir l'état actuel des articulations"""
        # Mettre à jour l'état actuel des articulations
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
                
                # Si une des articulations correspond à la pince, la mettre à jour
                if 'fr3_finger_joint1' in name.lower() or 'fr3_finger_joint2' in name.lower():
                    self.pince = abs(msg.position[i])  # Valeur absolue de l'ouverture de la pince

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

    def calcul_distance(self):
        distance = math.sqrt( (self.x - self.pose_objet_x)**2 + (self.y - self.pose_objet_y)**2 + (self.z - self.pose_objet_z)**2)
        if distance < 0.1 and self.pince <= 0.02:
            self.masse_obj = 0.2

    def send_force(self):
        """
        Calcule la force à appliquer en fonction des limites définies.
        La force est proportionnelle à la distance depuis les seuils et
        augmente quand on s'approche des limites physiques.
        Forces envoyées uniquement en mode vitesse.
        """
        if not self.controller_pose_received:
            return
            
        # Incrémenter le compteur pour affichage périodique
        self.count += 1
        
        # Vecteur pour stocker les forces de répulsion calculées
        repulsion_force = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [tx, ty, tz, rx, ry, rz]
        
        # Si on n'est pas en mode vitesse, envoyer des forces nulles
        if not self.in_speed_mode:
            # Create message to publish the force (toutes à zéro)
            force_message = Float32MultiArray()
            force_message.data = repulsion_force
            
            # Publish force message with zeros
            self.force_joystick_pub.publish(force_message)
            
            # Afficher l'état du mode tous les 1000 cycles
            if self.count % 1000 == 0:
                self.get_logger().debug("Mode Position: Aucune force appliquée")
            
            return
        
        # Si on est en mode vitesse, calculer les forces
        # Drapeaux pour indiquer si une articulation est en zone critique
        critical_axis = False
        warning_axis = False
        
        # Tableau pour stocker les valeurs cartésiennes (translation + angles d'Euler)
        cartesian_values = [
            self.controleur_x, self.controleur_y, self.controleur_z,
            self.euler_x, self.euler_y, self.euler_z
        ]

        # Calculer la distance et mettre à jour la masse de l'objet
        self.calcul_distance()

        # CALCULER L'ACCÉLÉRATION À CHAQUE CYCLE
        self.calcul_accel()
        
        # Vérifier chaque axe et calculer la force si nécessaire
        for axis in range(6):  # 6 axes: 3 translations + 3 rotations (euler)
            # Récupérer la position actuelle
            axis_position = cartesian_values[axis]
            
            # Récupérer les limites pour cet axe
            limits = self.value_for_deplacement[axis]
            lower_threshold, upper_threshold, max_limit, min_limit = limits
            
            # Calculer la force proportionnelle si l'axe est hors des seuils
            force = 0.0
            in_warning_zone = False
            in_critical_zone = False
            
            # Calcul de la force si en dehors des seuils
            if axis_position < lower_threshold:
                # Plus on s'éloigne du seuil inférieur, plus la force est importante
                distance = lower_threshold - axis_position
                range_val = lower_threshold - min_limit
                
                if range_val > 0:
                    normalized_distance = distance / range_val  # 0 à 1
                    
                    if axis < 3:  # Translation
                        # Récupérer l'accélération correspondante
                        accel = [self.accel_x, self.accel_y, self.accel_z][axis]
                        parroies = [self.repulsion_parroies_x, self.repulsion_parroies_y, self.repulsion_parroies_z][axis]
                        force = normalized_distance * self.max_force_by_axis[axis] + (self.masse + self.masse_obj) * accel + 1 + parroies

                    else:  # Rotation (euler_x, euler_y, euler_z)
                        force = self.stiffness_rotation * normalized_distance * self.max_torque_by_axis[axis-3]
                    
                    # Déterminer les zones d'avertissement et critiques
                    if normalized_distance > 0.25:
                        in_warning_zone = True
                    if normalized_distance > 0.6:
                        in_critical_zone = True
            
            elif axis_position > upper_threshold:
                # Plus on s'éloigne du seuil supérieur, plus la force est importante
                distance = axis_position - upper_threshold
                range_val = max_limit - upper_threshold
                
                if range_val > 0:
                    normalized_distance = distance / range_val  # 0 à 1
                    
                    if axis < 3:  # Translation
                        # Récupérer l'accélération correspondante
                        accel = [self.accel_x, self.accel_y, self.accel_z][axis]
                        parroies = [self.repulsion_parroies_x, self.repulsion_parroies_y, self.repulsion_parroies_z][axis]
                        force = - (normalized_distance * self.max_force_by_axis[axis]) + self.masse * accel - 1 + parroies
                    else:  # Rotation (euler_x, euler_y, euler_z)
                        force = - (self.stiffness_rotation * normalized_distance * self.max_torque_by_axis[axis-3])
                    
                    # Déterminer les zones d'avertissement et critiques
                    if normalized_distance > 0.25:
                        in_warning_zone = True
                    if normalized_distance > 0.6:
                        in_critical_zone = True
            
            # Limiter la force au maximum
            if axis < 3:  # Translation
                if abs(force) > self.max_force_by_axis[axis]:
                    force = self.max_force_by_axis[axis] if force > 0 else -self.max_force_by_axis[axis]
            else:  # Rotation (euler_x, euler_y, euler_z)
                if abs(force) > self.max_torque_by_axis[axis-3]:
                    force = self.max_torque_by_axis[axis-3] if force > 0 else -self.max_torque_by_axis[axis-3]
            
            # Assigner la force calculée au bon axe
            repulsion_force[axis] = force
            
            # Mettre à jour les drapeaux globaux
            if in_critical_zone:
                critical_axis = True
            if in_warning_zone:
                warning_axis = True
            
            # Log debug info every 1000 cycles
            if self.count % 1000 == 0:
                axis_name = ["X trans", "Y trans", "Z trans", "Roll", "Pitch", "Yaw"][axis]
                debug_str = f"Axe {axis_name}: "
                
                if axis < 3:
                    debug_str += f"Pos={axis_position:.3f}m Force={force:.3f}N"
                else:
                    debug_str += f"Pos={math.degrees(axis_position):.1f}° Force={force:.3f}Nm"
                
                if in_critical_zone:
                    debug_str += " [CRITIQUE]"
                elif in_warning_zone:
                    debug_str += " [ATTENTION]"
                
                self.get_logger().debug(debug_str)
        
        # Create message to publish the force
        force_message = Float32MultiArray()
        
        # Fill force data
        force_message.data = repulsion_force
        
        # Publish force message
        self.force_joystick_pub.publish(force_message)
        
        # Display forces every 1000 cycles
        if self.count % 100 == 0:
            self.get_logger().debug(f"Mode Vitesse - Forces appliquées: F[{repulsion_force[0]:.2f}, {repulsion_force[1]:.2f}, {repulsion_force[2]:.2f}] " +
                                  f"T[{repulsion_force[3]:.2f}, {repulsion_force[4]:.2f}, {repulsion_force[5]:.2f}]")
            
            # Status warning/critical
            if critical_axis:
                self.get_logger().debug("ATTENTION: Axes en zone CRITIQUE")
            elif warning_axis:
                self.get_logger().debug("Axes en zone d'ATTENTION")

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
        
        return roll, pitch, yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convertit des angles d'Euler (roll, pitch, yaw) en quaternion.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw
    
    def calcul_accel(self):
        """
        Calcule l'accélération basée sur les positions actuelles et précédentes.
        Cette fonction est maintenant appelée dans send_force() pour garantir un calcul systématique.
        """
        current_time = time.time()
        
        # Vérifier si les données du contrôleur sont trop anciennes
        if not self.controller_pose_received or (current_time - self.last_controller_update_time) > self.controller_data_timeout:
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
        if self.accel_x > 2:
            self.accel_x = -2.0
        elif self.accel_x < -2:
            self.accel_x = 2

        if self.accel_y > 2:
            self.accel_y = -2.0
        elif self.accel_y < -2:
            self.accel_y = 2

        if self.accel_z > 2:
            self.accel_z = -2.0
        elif self.accel_z < -2:
            self.accel_z = 2
        
        # Mettre à jour les positions précédentes
        self.distance_x_prev = self.controleur_x
        self.distance_y_prev = self.controleur_y
        self.distance_z_prev = self.controleur_z

        # Afficher l'information d'accélération périodiquement
        if self.count % 100 == 0:
            self.get_logger().debug(f"dt: {dt:.3f}s - Accélération suivant x: {self.accel_x:.2f}  y: {self.accel_y:.2f}  z: {self.accel_z:.2f}")
        
        return self.accel_x, self.accel_y, self.accel_z

def main(args=None):
    rclpy.init(args=args)
    node = Joystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
