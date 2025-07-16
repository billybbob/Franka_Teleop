#!/usr/bin/env python3
"""
Nœud ROS2 pour la téléopération d'un robot Franka Fr3 avec cinématique inverse.

Ce module implémente un solveur de cinématique inverse pour le robot Franka Fr3,
permettant le contrôle en vitesse cartésienne via un dispositif haptique Desktop 6D de Haption.
Il utilise la bibliothèque Pinocchio pour les calculs de cinématique et inclut des
fonctionnalités de sécurité pour éviter les collisions avec les parois de la cellule.

Auteur: Vincent Bassemayousse
Date: 07/10/2025
Version: 1.0
Licence: Apache 2.0

Dépendances:
    - ROS2 Humble
    - Pinocchio
    - NumPy
    - franka_description (package ROS2)

Topics:
    Publishers:
        - /joint_velocities (Float32MultiArray): Vitesses des 7 articulations du bras
        - /gripper_command_velocities (Float64): Vitesse de la pince
    
    Subscribers:
        - /joint_states (JointState): État actuel des articulations
        - /cmd_vel (Float32MultiArray): Commandes de vitesse cartésienne
        - /position_effecteur_robot (TFMessage): Position de l'effecteur final
        - /distance_robot_parroies (Float32MultiArray): Distances aux parois
"""

import pinocchio as pin
import numpy as np
import os
import subprocess
import tempfile
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage


class FrankaIKSolver(Node):
    """
    Nœud ROS2 pour la résolution de cinématique inverse du robot Franka Fr3.
    
    Cette classe implémente un solveur de cinématique inverse en temps réel pour
    le robot Franka Fr3, permettant le contrôle en vitesse cartésienne. Elle inclut
    des fonctionnalités de sécurité pour éviter les collisions avec les limites
    de l'espace de travail.
    
    Attributes:
        model (pin.Model): Modèle cinématique Pinocchio du robot
        data (pin.Data): Données de calcul Pinocchio
        ee_id (int): ID du frame de l'effecteur final
        current_joint_positions (np.ndarray): Positions articulaires actuelles
        Vx, Vy, Vz (float): Vitesses linéaires cartésiennes (m/s)
        Rx, Ry, Rz (float): Vitesses angulaires cartésiennes (rad/s)
        Vg (float): Vitesse de la pince (rad/s)
        marge_securite (float): Marge de sécurité pour les parois (m)
    """
    
    def __init__(self):
        """
        Initialise le nœud FrankaIKSolver.
        
        Configure le modèle Pinocchio, initialise les publishers/subscribers,
        et démarre le timer pour l'envoi périodique des commandes.
        
        Raises:
            FileNotFoundError: Si le fichier XACRO du robot n'est pas trouvé
            RuntimeError: Si la conversion XACRO vers URDF échoue
        """
        super().__init__('franka_ik_solver')
        
        # Charger le modèle URDF du Franka Fr3 à partir du fichier XACRO
        self.get_logger().info("Chargement du modèle Fr3...")
        urdf_string = self.load_urdf_from_xacro()
        
        try:
            # Créer un fichier temporaire pour stocker l'URDF
            # Nécessaire car Pinocchio requiert un fichier physique
            with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as tmp_file:
                tmp_file.write(urdf_string.encode())
                urdf_path = tmp_file.name
            
            # Création du modèle et des données pour Pinocchio
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            
            # Nettoyage du fichier temporaire
            os.unlink(urdf_path)
            
            # Obtenir l'ID de l'effecteur final (TCP - Tool Center Point)
            try:
                self.ee_id = self.model.getFrameId("fr3_hand_tcp")
                self.get_logger().info(f"Effecteur trouvé: fr3_hand_tcp (ID: {self.ee_id})")
            except:
                # Si le nom exact n'est pas trouvé, chercher un frame similaire
                self.get_logger().warn("Frame 'fr3_hand_tcp' non trouvé. Recherche d'un frame d'effecteur alternatif...")
                for i in range(self.model.nframes):
                    frame_name = self.model.frames[i].name
                    # Recherche de mots-clés typiques pour l'effecteur final
                    if 'hand' in frame_name.lower() or 'tcp' in frame_name.lower() or 'tool' in frame_name.lower() or 'ee' in frame_name.lower():
                        self.ee_id = i
                        self.get_logger().info(f"Utilisation de '{frame_name}' comme effecteur (ID: {self.ee_id})")
                        break
            
            # Affichage des informations du modèle chargé
            self.get_logger().info(f"Modèle chargé avec succès: {self.model.name}")
            self.get_logger().info(f"Nombre de degrés de liberté: {self.model.nq}")
            self.get_logger().info(f"Noms des articulations: {[self.model.names[i+1] for i in range(self.model.njoints-1)]}")

            # === CONFIGURATION DES PUBLISHERS ===
            
            # Publisher pour les vitesses des 7 articulations du bras (sans la pince)
            self.joint_velocities_pub = self.create_publisher(
                Float32MultiArray,
                '/joint_velocities',
                10
            )

            # Publisher pour la vitesse de la pince
            self.gripper_velocity_pub = self.create_publisher(
                Float64,
                '/gripper_command_velocities',
                10
            )

            # === CONFIGURATION DES SUBSCRIBERS ===
            
            # Subscriber pour l'état actuel des articulations
            self.joint_state_sub = self.create_subscription(
                JointState,
                '/NS_1/joint_states',     # Topic pour le robot réel
                # '/joint_states',            # Topic pour la simulation
                self.joint_state_callback,
                10
            )
            
            # Subscriber pour les vitesses cartésiennes du contrôleur haptique
            self.cmd_vel_sub = self.create_subscription(
                Float32MultiArray, 
                '/cmd_vel', 
                self.cmd_vel_callback, 
                10
            )

            # Subscriber pour la pose de l'effecteur du robot
            self.position_effecteur_robot_sub = self.create_subscription(
                TFMessage, 
                '/position_effecteur_robot', 
                self.position_effecteur_robot_callback, 
                10
            )

            # Subscriber pour les distances entre parois et robot (sécurité)
            self.distance_parroies_sub = self.create_subscription(
                Float32MultiArray,
                '/distance_robot_parroies',
                self.distance_parroies_callback,
                10
            )
            
            # === INITIALISATION DES VARIABLES D'ÉTAT ===
            
            # Mémoriser l'état actuel des articulations (configuration q)
            self.current_joint_positions = np.zeros(self.model.nq)
            
            # Initialisation des vitesses cartésiennes (seront mises à jour par cmd_vel)
            self.Vx = 0.0  # Vitesse linéaire en X (m/s)
            self.Vy = 0.0  # Vitesse linéaire en Y (m/s) 
            self.Vz = 0.0  # Vitesse linéaire en Z (m/s)
            self.Rx = 0.0  # Vitesse angulaire autour de X (rad/s)
            self.Ry = 0.0  # Vitesse angulaire autour de Y (rad/s)
            self.Rz = 0.0  # Vitesse angulaire autour de Z (rad/s)
            self.Vg = 0.0  # Vitesse de la pince/gâchette (rad/s)

            # Identification des articulations des doigts de la pince
            self.finger_joint_ids = []
            for i in range(1, self.model.njoints):
                joint_name = self.model.names[i]
                if 'fr3_finger_joint' in joint_name:
                    self.finger_joint_ids.append(i-1)  # -1 car Pinocchio compte à partir de 1
                    self.get_logger().info(f"Articulation de doigt trouvée: {joint_name} (ID: {i-1})")

            # === VARIABLES DE POSITION ET ORIENTATION ===
            
            # Position cartésienne actuelle de l'effecteur final
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            
            # Orientation actuelle (quaternion)
            self.qx = 0.0
            self.qy = 0.0
            self.qz = 0.0
            self.qw = 1.0  # Quaternion d'identité (pas de rotation)

            # === VARIABLES DE SÉCURITÉ ===
            
            # Distances aux parois pour éviter les collisions
            self.distance_parroies_x = float('inf')  # Distance à la paroi en X
            self.distance_parroies_y = float('inf')  # Distance à la paroi en Y
            self.distance_parroies_z = float('inf')  # Distance à la paroi en Z
            self.distances_received = False  # Flag indiquant si les distances ont été reçues
            
            # Distance de sécurité minimale (5 cm)
            self.marge_securite = 0.05  # en mètres
            
            # === TIMER POUR L'ENVOI PÉRIODIQUE DES COMMANDES ===
            
            # Timer pour envoyer les commandes de vitesse à 10Hz
            self.timer = self.create_timer(0.1, self.send_cartesian_velocities)
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors du chargement du modèle: {e}")
    
    def cmd_vel_callback(self, msg):
        """
        Callback pour recevoir les vitesses cartésiennes depuis le contrôleur haptique.
        
        Cette méthode traite les commandes de vitesse cartésienne envoyées par le
        contrôleur haptique Desktop 6D de Haption. Le message contient 6 ou 7 valeurs :
        - 3 vitesses linéaires (Vx, Vy, Vz) en m/s
        - 3 vitesses angulaires (Rx, Ry, Rz) en rad/s
        - 1 vitesse de pince optionnelle (Vg) en rad/s
        
        Args:
            msg (Float32MultiArray): Message contenant les vitesses cartésiennes
                msg.data[0-2]: Vitesses linéaires [Vx, Vy, Vz]
                msg.data[3-5]: Vitesses angulaires [Rx, Ry, Rz]
                msg.data[6]: Vitesse de pince (optionnelle)
        
        Note:
            Les vitesses sont exprimées dans le repère de base du robot.
        """
        try:
            # Vérifier que le message contient assez de données (au moins 6 pour les vitesses cartésiennes)
            if len(msg.data) < 6:
                self.get_logger().error(f"Message cmd_vel invalide: attendu au moins 6 valeurs, reçu {len(msg.data)}")
                return
            
            # Extraire les vitesses linéaires du message Float32MultiArray
            self.Vx = float(msg.data[0])  # Vitesse linéaire selon X
            self.Vy = float(msg.data[1])  # Vitesse linéaire selon Y
            self.Vz = float(msg.data[2])  # Vitesse linéaire selon Z
            
            # Extraire les vitesses angulaires
            self.Rx = float(msg.data[3])  # Vitesse angulaire autour de X
            self.Ry = float(msg.data[4])  # Vitesse angulaire autour de Y
            self.Rz = float(msg.data[5])  # Vitesse angulaire autour de Z
            
            # Extraire la vitesse de la pince si elle est disponible (7ème valeur)
            if len(msg.data) >= 7:
                self.Vg = float(msg.data[6])
                self.get_logger().debug(f"Vitesse de gâchette reçue: {self.Vg}")
            
            self.get_logger().debug(f"Nouvelles vitesses reçues: lin[{self.Vx}, {self.Vy}, {self.Vz}], "
                                  f"ang[{self.Rx}, {self.Ry}, {self.Rz}], gâchette[{self.Vg}]")
                                  
        except Exception as e:
            self.get_logger().error(f"Erreur lors du traitement du message cmd_vel: {e}")
    
    def load_urdf_from_xacro(self):
        """
        Charge le modèle URDF du robot Franka Fr3 à partir du fichier XACRO.
        
        Cette méthode localise le fichier XACRO du robot Franka Fr3 dans le package
        franka_description et le convertit en URDF utilisable par Pinocchio.
        
        Returns:
            str: Contenu du fichier URDF généré à partir du XACRO
            
        Raises:
            FileNotFoundError: Si le fichier XACRO n'existe pas
            RuntimeError: Si la conversion XACRO vers URDF échoue
            
        Note:
            Cette méthode utilise l'outil en ligne de commande 'xacro' pour
            effectuer la conversion. Assurez-vous que xacro est installé.
        """
        # Trouver le chemin vers le package franka_description
        franka_desc_path = get_package_share_directory('franka_description')
        xacro_path = os.path.join(franka_desc_path, 'robots', 'fr3', 'fr3.urdf.xacro')
        
        self.get_logger().info(f"Chemin vers le fichier XACRO: {xacro_path}")
        
        # Vérifier si le fichier existe
        if not os.path.exists(xacro_path):
            self.get_logger().error(f"Le fichier XACRO n'existe pas: {xacro_path}")
            raise FileNotFoundError(f"Le fichier XACRO n'existe pas: {xacro_path}")
        
        # Convertir XACRO en URDF en utilisant la commande xacro
        result = subprocess.run(
            ['xacro', xacro_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Vérifier le succès de la conversion
        if result.returncode != 0:
            self.get_logger().error(f"Erreur lors de la conversion XACRO: {result.stderr}")
            raise RuntimeError(f"Erreur lors de la conversion XACRO: {result.stderr}")
        
        return result.stdout
    
    def joint_state_callback(self, msg):
        """
        Callback pour recevoir et traiter l'état actuel des articulations.
        
        Cette méthode met à jour les positions articulaires actuelles utilisées
        pour les calculs de cinématique inverse. Elle fait correspondre les noms
        des articulations du message avec ceux du modèle Pinocchio.
        
        Args:
            msg (JointState): Message contenant l'état des articulations
                msg.name: Liste des noms des articulations
                msg.position: Liste des positions correspondantes (rad)
                msg.velocity: Liste des vitesses (optionnel)
                msg.effort: Liste des couples (optionnel)
        
        Note:
            Seules les articulations présentes dans le modèle Pinocchio sont
            prises en compte. Les autres sont ignorées silencieusement.
        """
        # Mettre à jour l'état actuel des articulations
        for i, name in enumerate(msg.name):
            # Trouver l'index de cette articulation dans le modèle Pinocchio
            try:
                idx = self.model.getJointId(name) - 1  # -1 car Pinocchio compte à partir de 1
                if idx >= 0 and idx < self.model.nq:
                    self.current_joint_positions[idx] = msg.position[i]
            except:
                # Ignorer les articulations qui ne sont pas dans le modèle
                pass
    
    def position_effecteur_robot_callback(self, msg):
        """
        Callback pour recevoir la position de l'effecteur final du robot.
        
        Cette méthode traite les messages de transformation contenant la pose
        actuelle de l'effecteur final du robot dans l'espace cartésien.
        
        Args:
            msg (TFMessage): Message de transformation contenant :
                msg.transforms[0].transform.translation: Position (x, y, z)
                msg.transforms[0].transform.rotation: Orientation (quaternion)
        
        Note:
            Cette information peut être utilisée pour des calculs de contrôle
            en position ou pour la surveillance de la pose du robot.
        """
        if len(msg.transforms) == 0:
            self.get_logger().error("Message de position robot reçu est vide (aucune transformation)")
            return
            
        # Récupérer la première transformation du message
        transform = msg.transforms[0]
        
        # Mettre à jour la position du robot (translation)
        self.x = float(transform.transform.translation.x)
        self.y = float(transform.transform.translation.y)
        self.z = float(transform.transform.translation.z)
        
        # Mettre à jour l'orientation du robot (quaternion)
        self.qx = float(transform.transform.rotation.x)
        self.qy = float(transform.transform.rotation.y)
        self.qz = float(transform.transform.rotation.z)
        self.qw = float(transform.transform.rotation.w)
        
        # Marquer que la pose du robot a été reçue
        self.robot_pose_received_ = True
        
        self.get_logger().debug(f"Position robot reçue: [{self.x}, {self.y}, {self.z}] " +
                                f"[{self.qx}, {self.qy}, {self.qz}, {self.qw}]")
        
        # Calculer et envoyer la nouvelle position si nous avons les deux positions
        # (robot et contrôleur haptique)
        if hasattr(self, 'in_pose_mode_') and self.in_pose_mode_ and \
           hasattr(self, 'controller_pose_received_') and self.controller_pose_received_:
            self.calculateAndSendCommand()

    def distance_parroies_callback(self, msg):
        """
        Callback pour recevoir les distances calculées par le nœud de sécurité.
        
        Cette méthode reçoit les distances minimales entre l'effecteur final
        et les parois de la cellule de travail, calculées par un nœud externe
        (distance_parois.py). Ces informations sont utilisées pour la sécurité.
        
        Args:
            msg (Float32MultiArray): Distances aux parois
                msg.data[0]: Distance minimale selon l'axe X (m)
                msg.data[1]: Distance minimale selon l'axe Y (m)
                msg.data[2]: Distance minimale selon l'axe Z (m)
        
        Note:
            Ces distances sont utilisées par la méthode limite_cellule()
            pour arrêter le robot s'il s'approche trop des limites.
        """
        if len(msg.data) < 3:
            self.get_logger().error(f"Message de distance invalide: attendu au moins 3 valeurs, reçu {len(msg.data)}")
            return
            
        # Mettre à jour les distances aux parois
        self.distance_parroies_x = float(msg.data[0])
        self.distance_parroies_y = float(msg.data[1])
        self.distance_parroies_z = float(msg.data[2])
        self.distances_received = True
        
        self.get_logger().debug(f"Distances reçues: X={self.distance_parroies_x:.3f}m, "
                              f"Y={self.distance_parroies_y:.3f}m, Z={self.distance_parroies_z:.3f}m")
        
    def limite_cellule(self):
        """
        Vérifie si l'effecteur final est trop proche des limites de la cellule.
        
        Cette méthode de sécurité vérifie si l'effecteur final du robot est
        à une distance inférieure à la marge de sécurité (5 cm) des parois.
        Si c'est le cas, elle arrête le robot et émet un message d'erreur.
        
        Returns:
            bool: True si le mouvement est autorisé, False si le robot doit s'arrêter
            
        Note:
            Cette fonction utilise les distances calculées par le nœud externe
            distance_parois.py. Si aucune distance n'a été reçue, le mouvement
            est autorisé par défaut (mode dégradé).
            
        Safety:
            - Marge de sécurité : 5 cm (self.marge_securite)
            - Arrêt automatique si limite dépassée
            - Messages d'alerte dans les logs
        """
        # Vérifier que nous avons reçu les distances du nœud distance_parois
        if not self.distances_received:
            self.get_logger().warn("Aucune distance reçue du nœud distance_parois, "
                                 "impossible de vérifier les limites de la cellule")
            return True  # Autoriser le mouvement par défaut si pas de données
        
        # Vérifier si l'effecteur est trop proche des limites
        trop_proche = False
        message = ""
        
        # Vérification sur l'axe X
        if self.distance_parroies_x < self.marge_securite:
            trop_proche = True
            message += f"Trop proche des parois sur l'axe X! (distance: {self.distance_parroies_x:.3f}m) "
        
        # Vérification sur l'axe Y
        if self.distance_parroies_y < self.marge_securite:
            trop_proche = True
            message += f"Trop proche des parois sur l'axe Y! (distance: {self.distance_parroies_y:.3f}m) "
        
        # Vérification sur l'axe Z
        if self.distance_parroies_z < self.marge_securite:
            trop_proche = True
            message += f"Trop proche des parois sur l'axe Z! (distance: {self.distance_parroies_z:.3f}m) "
        
        # Si trop proche, logger un message d'erreur et arrêter le robot
        if trop_proche:
            self.get_logger().error(f"ALERTE SÉCURITÉ: {message}")
            # Arrêter le robot en mettant toutes les vitesses à zéro
            self.set_cartesian_velocities(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return False  # Ne pas autoriser le mouvement
        
        return True  # Autoriser le mouvement

    def send_cartesian_velocities(self):
        """
        Calcule et envoie les vitesses articulaires correspondant aux vitesses cartésiennes.
        
        Cette méthode principale effectue les calculs de cinématique inverse pour
        convertir les vitesses cartésiennes (reçues du contrôleur haptique) en
        vitesses articulaires à envoyer au robot. Elle utilise la jacobienne
        et la pseudo-inverse pour résoudre le problème de cinématique différentielle.
        
        Algorithme:
            1. Vérification de la sécurité (limites de cellule)
            2. Calcul de la cinématique directe pour la configuration actuelle
            3. Construction du vecteur de vitesse cartésienne
            4. Calcul de la jacobienne du robot
            5. Résolution par pseudo-inverse avec amortissement
            6. Limitation des vitesses articulaires
            7. Publication des commandes
        
        Note:
            - Fréquence d'exécution : 10 Hz (définie par le timer)
            - Utilise un facteur d'amortissement pour éviter les singularités
            - Limite les vitesses articulaires pour la sécurité
        
        Safety:
            - Vérification des limites de cellule avant calcul
            - Limitation des vitesses articulaires maximales
            - Gestion des singularités par amortissement
        """
        # Vérifier qu'on a reçu au moins une fois l'état des articulations
        if np.all(self.current_joint_positions == 0):
            self.get_logger().warn("Aucune position articulaire reçue, impossible de calculer les vitesses")
            return
        
        # Vérifier les limites de la cellule avant de continuer (sécurité)
        if not self.limite_cellule():
            self.get_logger().warn("Mouvement arrêté : robot trop proche des limites de la cellule")
            return
        
        # Mettre à jour la cinématique directe pour la configuration actuelle
        q_current = self.current_joint_positions
        pin.forwardKinematics(self.model, self.data, q_current)
        pin.updateFramePlacement(self.model, self.data, self.ee_id)
        
        # Créer le vecteur de vitesse cartésienne [vx, vy, vz, wx, wy, wz]
        v_cart = np.array([self.Vx, self.Vy, self.Vz, self.Rx, self.Ry, self.Rz])
        
        # Calculer la Jacobienne géométrique de l'effecteur final
        # La jacobienne relie les vitesses articulaires aux vitesses cartésiennes
        J = pin.computeFrameJacobian(self.model, self.data, q_current, self.ee_id)
        
        # Facteur d'amortissement pour éviter les singularités
        # Méthode de Levenberg-Marquardt pour la stabilité numérique
        damp = 1e-6
        
        # Calculer les vitesses articulaires à partir des vitesses cartésiennes
        # Utilisation de la pseudo-inverse avec amortissement : dq = J^+ * v_cart
        J_pinv = np.linalg.pinv(J, rcond=damp)
        dq = J_pinv @ v_cart
        
        # Limiter les vitesses articulaires pour la sécurité du robot
        max_velocity = 0.5  # rad/s, ajustez selon les spécifications du robot
        dq = np.clip(dq, -max_velocity, max_velocity)
        
        # Créer une liste pour les vitesses articulaires (seulement le bras, pas la pince)
        # Le Fr3 a 7 degrés de liberté pour le bras + 2 pour la pince
        arm_velocities = list(dq[:7])  # Premières 7 articulations (bras)
        
        # Publier les vitesses du bras comme Float32MultiArray
        velocities_msg = Float32MultiArray()
        velocities_msg.data = arm_velocities
        self.joint_velocities_pub.publish(velocities_msg)
        
        # Publier la vitesse de la pince séparément sur le topic dédié
        gripper_msg = Float64()
        gripper_msg.data = self.Vg
        self.gripper_velocity_pub.publish(gripper_msg)
        
        # Log des vitesses calculées (pour debug)
        self.get_logger().debug(f"Vitesses cartésiennes: [{self.Vx}, {self.Vy}, {self.Vz}, "
                              f"{self.Rx}, {self.Ry}, {self.Rz}]")
        self.get_logger().debug(f"Vitesse gâchette: {self.Vg}")
        self.get_logger().debug(f"Vitesses articulaires calculées (bras): {arm_velocities}")
    
    def target_pose_callback(self, msg):
        """
        Callback pour recevoir une pose cible et calculer les vitesses articulaires.
        
        Cette méthode alternative permet un contrôle en position plutôt qu'en vitesse.
        Elle calcule l'erreur entre la pose actuelle et la pose cible, puis
        génère les vitesses articulaires nécessaires pour réduire cette erreur.
        
        Args:
            msg: Message de pose cible contenant :
                msg.position: Position cible (x, y, z)
                msg.orientation: Orientation cible (quaternion)
        
        Algorithm:
            1. Extraction de la pose cible
            2. Calcul de la cinématique directe actuelle
            3. Calcul de l'erreur spatiale (SE3)
            4. Résolution de la cinématique inverse différentielle
            5. Application d'un gain pour contrôler la vitesse d'approche
        
        Note:
            Cette méthode n'est pas utilisée dans le mode de téléopération
            en vitesse, mais peut être utile pour des mouvements point-à-point.
        """
        # Extraire la position et l'orientation de la pose cible
        target_position = np.array([msg.position.x, msg.position.y, msg.position.z])
        
        # Convertir le quaternion en matrice de rotation
        q = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z])
        target_orientation = pin.Quaternion(q).toRotationMatrix()
        
        # Création de la matrice de transformation homogène cible
        target_SE3 = pin.SE3(target_orientation, target_position)
        
        # Calculer la FK pour la configuration actuelle
        q_current = self.current_joint_positions
        pin.forwardKinematics(self.model, self.data, q_current)
        pin.updateFramePlacement(self.model, self.data, self.ee_id)
        current_SE3 = self.data.oMf[self.ee_id]
        
        # Calculer l'erreur spatiale (différence entre pose actuelle et cible)
        # Utilisation du logarithme de SE(3) pour obtenir l'erreur dans l'espace des vitesses
        error = pin.log(current_SE3.inverse() * target_SE3).vector
        
        # Calculer la Jacobienne pour la configuration actuelle
        J = pin.computeFrameJacobian(self.model, self.data, q_current, self.ee_id)
        
        # Facteur d'amortissement pour éviter les singularités
        damp = 1e-6
        
        # Calculer les vitesses articulaires nécessaires pour se diriger vers la cible
        J_pinv = np.linalg.pinv(J, rcond=damp)
        dq = J_pinv @ error
        
        # Facteur de gain pour ajuster la vitesse d'approche
        gain = 0.2  # Ajustez selon vos besoins (plus petit = approche plus lente)
        dq *= gain

        # Limiter les vitesses articulaires
        max_velocity = 0.5  # rad/s, ajustez selon les spécifications du robot
        dq = np.clip(dq, -max_velocity, max_velocity)
        
        # Créer une liste pour les vitesses articulaires (seulement le bras)
        arm_velocities = list(dq[:7])  # 7 articulations principales
        
        # Publier les vitesses du bras
        velocities_msg = Float32MultiArray()
        velocities_msg.data = arm_velocities
        self.joint_velocities_pub.publish(velocities_msg)
        
        # Publier la vitesse de la pince séparément
        gripper_msg = Float64()
        gripper_msg.data = self.Vg
        self.gripper_velocity_pub.publish(gripper_msg)
    
    def set_cartesian_velocities(self, vx, vy, vz, rx, ry, rz, vg=None):
        """
        Définit les vitesses cartésiennes à utiliser pour le contrôle du robot.
        
        Cette méthode utilitaire permet de définir manuellement les vitesses
        cartésiennes sans passer par le callback cmd_vel. Utile pour les
        tests, l'arrêt d'urgence, ou le contrôle programmatique.
        
        Args:
            vx (float): Vitesse linéaire selon X (m/s)
            vy (float): Vitesse linéaire selon Y (m/s)
            vz (float): Vitesse linéaire selon Z (m/s)
            rx (float): Vitesse angulaire autour de X (rad/s)
            ry (float): Vitesse angulaire autour de Y (rad/s)
            rz (float): Vitesse angulaire autour de Z (rad/s)
            vg (float, optional): Vitesse de la pince (rad/s)
        
        Example:
            # Arrêt d'urgence
            solver.set_cartesian_velocities(0, 0, 0, 0, 0, 0, 0)
            
            # Mouvement linéaire en X à 1 cm/s
            solver.set_cartesian_velocities(0.01, 0, 0, 0, 0, 0)
        """
        self.Vx = vx
        self.Vy = vy
        self.Vz = vz
        self.Rx = rx
        self.Ry = ry
        self.Rz = rz
        
        # Mettre à jour la vitesse de la pince si spécifiée
        if vg is not None:
            self.Vg = vg
            self.get_logger().debug(f"Nouvelles vitesses cartésiennes définies: [{vx}, {vy}, {vz}, "
                                  f"{rx}, {ry}, {rz}], gâchette: {vg}")
        else:
            self.get_logger().debug(f"Nouvelles vitesses cartésiennes définies: [{vx}, {vy}, {vz}, "
                                  f"{rx}, {ry}, {rz}]")
    
    def forward_kinematics(self, q):
        """
        Calcule la cinématique directe pour une configuration articulaire donnée.
        
        Cette méthode calcule la position et l'orientation de l'effecteur final
        pour une configuration articulaire donnée. Utile pour la planification
        de trajectoire, la vérification de poses, ou l'analyse de l'espace de travail.
        
        Args:
            q (np.ndarray): Vecteur de configuration articulaire (angles en radians)
                            Taille : [self.model.nq] (généralement 7 pour le Fr3)
        
        Returns:
            tuple: (position, orientation)
                position (np.ndarray): Position cartésienne [x, y, z] en mètres
                orientation (np.ndarray): Matrice de rotation 3x3
        
        Example:
            # Configuration articulaire de test
            q_test = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
            pos, rot = solver.forward_kinematics(q_test)
            print(f"Position: {pos}")
            print(f"Orientation:\n{rot}")
        
        Note:
            Cette méthode ne modifie pas l'état interne du solveur, contrairement
            aux méthodes de callback qui mettent à jour self.current_joint_positions.
        """
        # Copie de la configuration actuelle pour ne pas la modifier
        q_copy = q.copy()
        
        # Calculer la cinématique directe pour la configuration donnée
        pin.forwardKinematics(self.model, self.data, q_copy)
        pin.updateFramePlacement(self.model, self.data, self.ee_id)
        
        # Obtenir la position et l'orientation de l'effecteur final
        ee_position = self.data.oMf[self.ee_id].translation
        ee_orientation = self.data.oMf[self.ee_id].rotation
        
        return ee_position, ee_orientation
    
    def inverse_kinematics(self, target_position, target_orientation, q_init=None):
        """
        Calcule la cinématique inverse pour atteindre une pose cible.
        
        Cette méthode résout le problème de cinématique inverse pour trouver
        la configuration articulaire permettant d'atteindre une pose donnée.
        Elle utilise l'algorithme de Newton-Raphson avec la méthode de
        Levenberg-Marquardt pour la stabilité numérique.
        
        Args:
            target_position (np.ndarray): Position cible [x, y, z] en mètres
            target_orientation (np.ndarray): Matrice de rotation cible 3x3
            q_init (np.ndarray, optional): Configuration initiale pour l'optimisation.
                                         Si None, utilise la configuration actuelle.
        
        Returns:
            np.ndarray: Configuration articulaire solution (angles en radians)
                       Taille : [self.model.nq]
        
        Algorithm:
            1. Initialisation avec q_init ou configuration actuelle
            2. Boucle d'optimisation Newton-Raphson :
               a. Calcul de la cinématique directe
               b. Calcul de l'erreur en pose
               c. Calcul de la Jacobienne
               d. Mise à jour par pseudo-inverse
               e. Normalisation et application des limites
            3. Convergence ou nombre max d'itérations atteint
        
        Parameters (Algorithm):
            eps (float): Tolérance de convergence (1e-4)
            max_iter (int): Nombre maximum d'itérations (100)
            damp (float): Facteur d'amortissement pour singularités (1e-6)
        
        Example:
            # Pose cible : position + orientation
            target_pos = np.array([0.5, 0.2, 0.4])
            target_rot = np.eye(3)  # Pas de rotation
            
            # Résolution IK
            q_solution = solver.inverse_kinematics(target_pos, target_rot)
            
            # Vérification
            pos, rot = solver.forward_kinematics(q_solution)
            error = np.linalg.norm(pos - target_pos)
            print(f"Erreur de position: {error:.6f} m")
        
        Note:
            - La convergence n'est pas garantie pour toutes les poses
            - Certaines poses peuvent être hors de l'espace de travail
            - Les singularités peuvent empêcher la convergence
            - La solution dépend de la configuration initiale
        
        Warning:
            Cette méthode peut être coûteuse en calcul pour des utilisations
            fréquentes. Pour la téléopération temps réel, préférer le contrôle
            en vitesse avec send_cartesian_velocities().
        """
        # Si aucune config initiale n'est fournie, utiliser la position actuelle
        if q_init is None:
            q_init = self.current_joint_positions
        
        # Création de la matrice de transformation homogène cible
        target_SE3 = pin.SE3(target_orientation, target_position)
        
        # Paramètres d'optimisation
        eps = 1e-4          # Tolérance de convergence
        max_iter = 100      # Nombre maximum d'itérations
        damp = 1e-6         # Facteur d'amortissement (Levenberg-Marquardt)
        
        # Initialisation de la solution avec la configuration initiale
        q_result = q_init.copy()
        
        # Boucle d'optimisation Newton-Raphson
        for i in range(max_iter):
            # Calculer la FK pour la configuration actuelle
            pin.forwardKinematics(self.model, self.data, q_result)
            pin.updateFramePlacement(self.model, self.data, self.ee_id)
            current_SE3 = self.data.oMf[self.ee_id]
            
            # Calculer l'erreur (différence entre pose actuelle et cible)
            # Utilisation du logarithme de SE(3) pour l'erreur spatiale
            error = pin.log(current_SE3.inverse() * target_SE3).vector
            
            # Test de convergence
            if np.linalg.norm(error) < eps:
                self.get_logger().info(f"IK convergée après {i} itérations")
                break
                
            # Calculer la Jacobienne pour la configuration actuelle
            J = pin.computeFrameJacobian(self.model, self.data, q_result, self.ee_id)
            
            # Mise à jour de la configuration (méthode de Levenberg-Marquardt)
            # Utilisation de la pseudo-inverse avec amortissement pour la stabilité
            J_pinv = np.linalg.pinv(J, rcond=damp)
            dq = J_pinv @ error
            q_result += dq
            
            # Normaliser les angles et appliquer les limites articulaires
            # Pinocchio gère automatiquement la normalisation des angles
            q_result = pin.normalize(self.model, q_result)
            
            # Si nous atteignons la dernière itération sans converger
            if i == max_iter - 1:
                self.get_logger().warn("IK n'a pas convergé après le nombre max d'itérations")
                self.get_logger().warn(f"Erreur finale: {np.linalg.norm(error):.6f}")
        
        return q_result

def main(args=None):
    """
    Point d'entrée principal du nœud ROS2.
    
    Cette fonction initialise le système ROS2, crée une instance du solveur
    de cinématique inverse, et lance la boucle principale d'exécution.
    
    Args:
        args: Arguments de ligne de commande (optionnel)
    
    Flow:
        1. Initialisation ROS2
        2. Création du nœud FrankaIKSolver
        3. Boucle d'exécution (spin)
        4. Nettoyage et arrêt propre
    
    Usage:
        python3 franka_ik_solver.py
        ou
        ros2 run [package_name] franka_ik_solver.py
    
    KeyboardInterrupt:
        Le programme peut être arrêté proprement avec Ctrl+C
    """
    # Initialisation du système ROS2
    rclpy.init(args=args)
    
    # Création de l'instance du solveur de cinématique inverse
    franka_solver = FrankaIKSolver()
    
    try:
        # Lancement de la boucle principale ROS2
        # Cette boucle traite les callbacks et maintient le nœud actif
        rclpy.spin(franka_solver)
        
    except KeyboardInterrupt:
        # Gestion de l'arrêt par Ctrl+C
        print("\nArrêt demandé par l'utilisateur (Ctrl+C)")
        
    finally:
        # Nettoyage et arrêt propre
        franka_solver.destroy_node()
        rclpy.shutdown()
        print("Nœud FrankaIKSolver arrêté proprement")


if __name__ == '__main__':
    main()