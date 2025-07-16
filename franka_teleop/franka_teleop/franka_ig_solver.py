#!/usr/bin/env python3
"""
Module de résolution de cinématique inverse pour robot Franka Fr3
============================================================

Ce module implémente un nœud ROS2 pour la résolution de la cinématique inverse
d'un robot Franka Fr3 dans le cadre d'un système de téléopération avec contrôleur
haptique Desktop 6D de Haption.

Le nœud utilise la bibliothèque Pinocchio pour les calculs de cinématique et
intègre des fonctions de sécurité pour éviter les collisions avec les parois
de la cellule robotique.

Auteur: Vincent Bassemayousse
Date: 07/10/2025
Version: 1.0
Licence: Apache 2.0
Dépendances:
    - ROS2 Humble
    - Pinocchio
    - NumPy
    - franka_description (package ROS2)
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
from std_msgs.msg import Float32MultiArray, Float64, Bool
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_msgs.msg import TFMessage


class FrankaMGSolver(Node):
    """
    Nœud ROS2 pour la résolution de cinématique inverse du robot Franka Fr3.
    
    Cette classe implémente un solveur de modèle géométrique inverse (MGI) pour
    un robot Franka Fr3, avec intégration de fonctions de sécurité pour éviter
    les collisions avec les limites de la cellule robotique.
    
    Le nœud reçoit des poses cibles via des messages Float32MultiArray et calcule
    les configurations articulaires correspondantes en utilisant la méthode de
    Levenberg-Marquardt.
    
    Attributes:
        model (pin.Model): Modèle cinématique Pinocchio du robot
        data (pin.Data): Données de calcul Pinocchio
        ee_id (int): Identifiant du frame de l'effecteur final
        current_joint_positions (np.ndarray): Positions articulaires actuelles
        x, y, z (float): Position actuelle de l'effecteur final
        distance_parroies_x/y/z (float): Distances aux parois de la cellule
        marge_securite (float): Marge de sécurité minimale (5 cm)
    """
    
    def __init__(self):
        """
        Initialise le nœud FrankaMGSolver.
        
        Configure le modèle cinématique, les publishers/subscribers ROS2,
        et initialise les paramètres de sécurité et de calcul MGI.
        """
        super().__init__('franka_ig_solver')
        
        # ═══════════════════════════════════════════════════════════════
        # INITIALISATION DU MODÈLE CINÉMATIQUE
        # ═══════════════════════════════════════════════════════════════
        
        # Charger le modèle URDF du Franka Fr3 à partir du fichier XACRO
        self.get_logger().info("Chargement du modèle Fr3...")
        urdf_string = self.load_urdf_from_xacro()
        
        try:
            # Créer un fichier temporaire pour stocker l'URDF généré
            with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as tmp_file:
                tmp_file.write(urdf_string.encode())
                urdf_path = tmp_file.name
            
            # Création du modèle et des données pour Pinocchio
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            
            # Nettoyage du fichier temporaire
            os.unlink(urdf_path)
            
            # ═══════════════════════════════════════════════════════════
            # RECHERCHE DE L'EFFECTEUR FINAL
            # ═══════════════════════════════════════════════════════════
            
            # Obtenir l'ID de l'effecteur final (TCP - Tool Center Point)
            try:
                self.ee_id = self.model.getFrameId("fr3_hand_tcp")
                self.get_logger().info(f"Effecteur trouvé: fr3_hand_tcp (ID: {self.ee_id})")
            except:
                # Si le nom exact n'est pas trouvé, chercher un frame similaire
                self.get_logger().warn("Frame 'fr3_hand_tcp' non trouvé. Recherche d'un frame d'effecteur alternatif...")
                for i in range(self.model.nframes):
                    frame_name = self.model.frames[i].name
                    # Recherche de mots-clés typiques pour un effecteur final
                    if any(keyword in frame_name.lower() for keyword in ['hand', 'tcp', 'tool', 'ee']):
                        self.ee_id = i
                        self.get_logger().info(f"Utilisation de '{frame_name}' comme effecteur (ID: {self.ee_id})")
                        break
            
            # Affichage des informations du modèle pour débogage
            self.get_logger().info(f"Modèle chargé avec succès: {self.model.name}")
            self.get_logger().info(f"Nombre de degrés de liberté: {self.model.nq}")
            self.get_logger().info(f"Noms des articulations: {[self.model.names[i+1] for i in range(self.model.njoints-1)]}")

            # ═══════════════════════════════════════════════════════════
            # CONFIGURATION DES PUBLISHERS ROS2
            # ═══════════════════════════════════════════════════════════
            
            # Publisher pour les positions articulaires calculées par MGI
            self.joint_positions_pub = self.create_publisher(
                Float32MultiArray,
                '/joint_positions',
                10
            )
            
            # Publisher pour les commandes de la pince (gripper)
            self.gripper_pub = self.create_publisher(
                Float64,
                '/gripper_command_positions',
                10
            )
            
            # Publisher pour l'état de sécurité de la cellule robotique
            self.safety_status_pub = self.create_publisher(
                Bool,
                '/cell_safety_status',
                10
            )

            # ═══════════════════════════════════════════════════════════
            # CONFIGURATION DES SUBSCRIBERS ROS2
            # ═══════════════════════════════════════════════════════════
            
            # Subscriber pour l'état actuel des articulations du robot
            self.joint_state_sub = self.create_subscription(
                JointState,
                '/NS_1/joint_states',     # Topic pour le robot réel
                # '/joint_states',            # Topic pour la simulation
                self.joint_state_callback,
                10)
            
            # Subscriber pour les poses cibles envoyées par le contrôleur haptique
            self.cmd_pose_sub = self.create_subscription(
                Float32MultiArray, 
                '/cmd_pose', 
                self.cmd_pose_callback, 
                10
            )
            
            # Subscriber pour la position de l'effecteur via les transformations TF
            self.position_effecteur_robot_sub = self.create_subscription(
                TFMessage, 
                '/position_effecteur_robot', 
                self.position_effecteur_robot_callback, 
                10
            )

            # Subscriber pour les distances calculées entre robot et parois
            self.distance_parroies_sub = self.create_subscription(
                Float32MultiArray,
                '/distance_robot_parroies',
                self.distance_parroies_callback,
                10
            )
            
            # ═══════════════════════════════════════════════════════════
            # CONFIGURATION TF ET ÉTAT INITIAL
            # ═══════════════════════════════════════════════════════════
            
            # Setup TF listener pour obtenir les transformations
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            
            # Mémoriser l'état actuel des articulations (7 DoF pour Fr3)
            self.current_joint_positions = np.zeros(self.model.nq)

            # Configuration initiale du robot (position de repos typique)
            # Ces valeurs correspondent à une pose neutre du Fr3
            self.current_joint_positions = np.array([
                -2.735252171512916e-07,  # Joint 1: Base rotation
                -0.7854108458715359,     # Joint 2: Shoulder pitch (~-45°)
                2.7657099024706256e-12,  # Joint 3: Shoulder roll
                -2.356229067447648,      # Joint 4: Elbow pitch (~-135°)
                -4.253565360196798e-06,  # Joint 5: Wrist pitch
                1.5708193814872145,      # Joint 6: Wrist roll (~90°)
                0.0                      # Joint 7: Wrist yaw
            ])
            
            # ═══════════════════════════════════════════════════════════
            # PARAMÈTRES DE CALCUL MGI
            # ═══════════════════════════════════════════════════════════
            
            # Paramètres configurables pour la résolution MGI
            self.mgi_max_iter = self.declare_parameter('mgi_max_iter', 100).value
            self.mgi_epsilon = self.declare_parameter('mgi_epsilon', 1e-4).value
            self.mgi_damping = self.declare_parameter('mgi_damping', 1e-1).value
            
            # ═══════════════════════════════════════════════════════════
            # VARIABLES D'ÉTAT ET DE SÉCURITÉ
            # ═══════════════════════════════════════════════════════════
            
            # Position actuelle de l'effecteur final dans l'espace cartésien
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.robot_pose_received_ = False  # Flag pour savoir si la pose a été reçue

            self.prev_error_norm = 0.0

            # Distances entre les parois de la cellule et l'effecteur du robot
            # Initialisées à l'infini pour la sécurité (pas de mouvement si pas de données)
            self.distance_parroies_x = float('inf')
            self.distance_parroies_y = float('inf')
            self.distance_parroies_z = float('inf')
            self.distances_received = False  # Flag pour savoir si les distances ont été reçues
            
            # Distance de sécurité minimale (5 cm) pour éviter les collisions
            self.marge_securite = 0.05  # en mètres
            
            # Timer pour vérifier régulièrement les limites de sécurité
            self.safety_timer = self.create_timer(0.1, self.check_cell_limits)  # Vérification toutes les 100ms
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors du chargement du modèle: {e}")
    
    def load_urdf_from_xacro(self):
        """
        Charge et convertit le modèle URDF à partir du fichier XACRO du Fr3.
        
        Cette méthode localise le fichier XACRO du robot Franka Fr3 dans le package
        franka_description et utilise l'outil xacro pour le convertir en URDF.
        
        Returns:
            str: Contenu URDF généré à partir du fichier XACRO
            
        Raises:
            FileNotFoundError: Si le fichier XACRO n'est pas trouvé
            RuntimeError: Si la conversion XACRO échoue
        """
        # Trouver le chemin vers le package franka_description
        franka_desc_path = get_package_share_directory('franka_description')
        xacro_path = os.path.join(franka_desc_path, 'robots', 'fr3', 'fr3.urdf.xacro')
        
        self.get_logger().info(f"Chemin vers le fichier XACRO: {xacro_path}")
        
        # Vérifier si le fichier existe
        if not os.path.exists(xacro_path):
            self.get_logger().error(f"Le fichier XACRO n'existe pas: {xacro_path}")
            raise FileNotFoundError(f"Le fichier XACRO n'existe pas: {xacro_path}")
        
        # Convertir XACRO en URDF en utilisant la commande système xacro
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
        Callback pour traiter les états articulaires reçus du robot.
        
        Met à jour les positions articulaires actuelles utilisées comme point
        de départ pour les calculs de cinématique inverse.
        
        Args:
            msg (JointState): Message contenant les noms et positions des articulations
        """
        # Parcourir tous les joints reçus dans le message
        for i, name in enumerate(msg.name):
            try:
                # Trouver l'index de cette articulation dans le modèle Pinocchio
                # Note: Pinocchio compte à partir de 1, d'où le -1
                idx = self.model.getJointId(name) - 1
                
                # Vérifier que l'index est valide et mettre à jour la position
                if 0 <= idx < self.model.nq:
                    self.current_joint_positions[idx] = msg.position[i]
            except:
                # Ignorer silencieusement les articulations non trouvées
                # (peut inclure des joints de la pince ou autres)
                pass
    
    def position_effecteur_robot_callback(self, msg):
        """
        Callback pour traiter la position de l'effecteur reçue via TF.
        
        Met à jour la position cartésienne actuelle de l'effecteur final
        en extrayant les informations des messages de transformation TF.
        
        Args:
            msg (TFMessage): Message contenant les transformations TF
        """
        try:
            # Parcourir toutes les transformations dans le message
            for transform in msg.transforms:
                # Vérifier si c'est la transformation de l'effecteur final
                # Plusieurs noms possibles selon la configuration
                if (transform.child_frame_id == 'fr3_hand_tcp' or 
                    transform.child_frame_id == 'end_effector' or 
                    'hand' in transform.child_frame_id or 
                    'tcp' in transform.child_frame_id):
                    
                    # Extraire la position de la transformation
                    self.x = transform.transform.translation.x
                    self.y = transform.transform.translation.y
                    self.z = transform.transform.translation.z
                    self.robot_pose_received_ = True
                    break
                    
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la mise à jour de la position de l'effecteur: {e}")
    
    def update_current_ee_position(self):
        """
        Met à jour la position de l'effecteur via calcul de cinématique directe.
        
        Cette méthode est conservée comme fallback si les données TF ne sont pas
        disponibles. Elle calcule la position de l'effecteur à partir des positions
        articulaires actuelles en utilisant le modèle géométrique direct.
        
        Note:
            Cette méthode n'est plus utilisée automatiquement depuis l'implémentation
            de la réception de position via TF, mais reste disponible pour le débogage.
        """
        try:
            # Préparer le vecteur de configuration (ajouter positions des doigts si nécessaire)
            current_q = self.current_joint_positions.copy()
            if current_q.shape[0] == 7:
                # Ajouter les positions des doigts de la pince (ouverture de 1 cm chacun)
                current_q = np.concatenate([current_q, [0.01, 0.01]])
            
            # Calculer la cinématique directe
            pin.forwardKinematics(self.model, self.data, current_q)
            pin.updateFramePlacement(self.model, self.data, self.ee_id)
            
            # Extraire et stocker la position de l'effecteur
            self.x = self.data.oMf[self.ee_id].translation[0]
            self.y = self.data.oMf[self.ee_id].translation[1]
            self.z = self.data.oMf[self.ee_id].translation[2]
            self.robot_pose_received_ = True
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors du calcul de la position de l'effecteur: {e}")

    def distance_parroies_callback(self, msg):
        """
        Callback pour traiter les distances aux parois de la cellule.
        
        Reçoit et stocke les distances calculées par le nœud distance_parois.py
        entre l'effecteur du robot et les parois de la cellule robotique.
        
        Args:
            msg (Float32MultiArray): Distances [x, y, z] aux parois de la cellule
        """
        # Vérifier que le message contient au moins 3 valeurs (x, y, z)
        if len(msg.data) < 3:
            self.get_logger().error(f"Message de distance invalide: attendu au moins 3 valeurs, reçu {len(msg.data)}")
            return
            
        # Stocker les distances pour les trois axes
        self.distance_parroies_x = float(msg.data[0])
        self.distance_parroies_y = float(msg.data[1])
        self.distance_parroies_z = float(msg.data[2])
        self.distances_received = True
        
        # Log de débogage pour surveiller les distances
        self.get_logger().debug(f"Distances reçues: X={self.distance_parroies_x:.3f}m, "
                               f"Y={self.distance_parroies_y:.3f}m, Z={self.distance_parroies_z:.3f}m")
    
    def limite_cellule(self):
        """
        Vérifie si l'effecteur respecte les limites de sécurité de la cellule.
        
        Utilise les distances calculées par le nœud distance_parois.py pour
        déterminer si l'effecteur du robot est trop proche des parois de la cellule
        robotique. Publie également l'état de sécurité sur le topic correspondant.
        
        Returns:
            bool: True si le robot est dans une zone sûre, False sinon
        """
        # Vérifier que nous avons reçu les données de distance
        if not self.distances_received:
            self.get_logger().warn("Distances des parois non reçues, impossible de vérifier les limites de la cellule")
            return True  # Autoriser le mouvement par défaut si pas de données
        
        # Variables de contrôle pour la sécurité
        trop_proche = False
        message = ""
        
        # ═══════════════════════════════════════════════════════════════
        # VÉRIFICATION DES DISTANCES DE SÉCURITÉ
        # ═══════════════════════════════════════════════════════════════
        
        # Vérification sur l'axe X (avant/arrière)
        if self.distance_parroies_x < self.marge_securite:
            trop_proche = True
            message += f"Trop proche des parois X! (distance: {self.distance_parroies_x:.3f}m) "
        
        # Vérification sur l'axe Y (gauche/droite)
        if self.distance_parroies_y < self.marge_securite:
            trop_proche = True
            message += f"Trop proche des parois Y! (distance: {self.distance_parroies_y:.3f}m) "
        
        # Vérification sur l'axe Z (haut/bas)
        if self.distance_parroies_z < self.marge_securite:
            trop_proche = True
            message += f"Trop proche des parois Z! (distance: {self.distance_parroies_z:.3f}m) "
        
        # ═══════════════════════════════════════════════════════════════
        # GESTION DES ALERTES DE SÉCURITÉ
        # ═══════════════════════════════════════════════════════════════
        
        # Créer et publier le message d'état de sécurité
        safety_msg = Bool()
        
        if trop_proche:
            # Danger détecté - arrêter le robot
            self.get_logger().error(f"ALERTE SÉCURITÉ: {message}")
            safety_msg.data = False
            self.safety_status_pub.publish(safety_msg)
            return False  # Bloquer tout mouvement
        else:
            # Zone sûre - autoriser les mouvements
            safety_msg.data = True
            self.safety_status_pub.publish(safety_msg)
            return True
    
    def check_cell_limits(self):
        """
        Fonction de vérification périodique des limites de sécurité.
        
        Appelée régulièrement par un timer ROS2 pour surveiller en continu
        les limites de la cellule robotique, même quand aucune commande
        n'est en cours de traitement.
        """
        self.limite_cellule()
    
    def is_target_safe(self, target_position):
        """
        Vérifie si une position cible est sûre avant d'exécuter le mouvement.
        
        Estime si le mouvement vers la position cible risque de violer les
        limites de sécurité de la cellule en extrapolant les distances actuelles.
        
        Args:
            target_position (np.ndarray): Position cible [x, y, z] en mètres
            
        Returns:
            bool: True si la position cible est estimée sûre, False sinon
            
        Note:
            Cette estimation est simplifiée et suppose un déplacement direct.
            Une vérification plus précise nécessiterait une simulation complète
            de la trajectoire.
        """
        # Vérifier que nous avons les données nécessaires
        if not self.distances_received or not self.robot_pose_received_:
            self.get_logger().warn("Impossible de vérifier la sécurité de la cible: données insuffisantes")
            return True  # Par défaut, autoriser si on ne peut pas vérifier
        
        # Calculer le vecteur de déplacement prévu
        current_pos = np.array([self.x, self.y, self.z])
        target_pos = np.array(target_position)
        deplacement = target_pos - current_pos
        
        # Estimer les nouvelles distances après le mouvement
        # Note: Cette estimation suppose que les parois sont perpendiculaires aux axes
        estimated_dist_x = max(0, self.distance_parroies_x - abs(deplacement[0]))
        estimated_dist_y = max(0, self.distance_parroies_y - abs(deplacement[1]))
        estimated_dist_z = max(0, self.distance_parroies_z - abs(deplacement[2]))
        
        # Vérifier si les distances estimées respectent la marge de sécurité
        if (estimated_dist_x < self.marge_securite or 
            estimated_dist_y < self.marge_securite or 
            estimated_dist_z < self.marge_securite):
            
            self.get_logger().warn(f"Position cible potentiellement dangereuse. Distances estimées: "
                                 f"X={estimated_dist_x:.3f}m, Y={estimated_dist_y:.3f}m, Z={estimated_dist_z:.3f}m")
            return False
        
        return True
    
    def cmd_pose_callback(self, msg):
        """
        Callback principal pour traiter les commandes de pose du contrôleur haptique.
        
        Reçoit une pose cible (position + orientation + commande pince) depuis
        le contrôleur haptique Desktop 6D, vérifie la sécurité, et calcule les
        angles articulaires correspondants via cinématique inverse.
        
        Args:
            msg (Float32MultiArray): 
                [0:3] - Position XYZ (mètres)
                [3:7] - Quaternion XYZW (orientation) 
                [7] - Commande pince (optionnel)
                
        Format du message attendu:
            - msg.data[0:3]: Position cartésienne [x, y, z] en mètres
            - msg.data[3:7]: Quaternion [qx, qy, qz, qw] pour l'orientation
            - msg.data[7]: Valeur de commande de la pince (optionnel)
        """
        # ═══════════════════════════════════════════════════════════════
        # VALIDATION DU MESSAGE D'ENTRÉE
        # ═══════════════════════════════════════════════════════════════
        
        if len(msg.data) < 7:
            self.get_logger().error(f"Message de pose invalide: attendu au moins 7 valeurs "
                                  f"(position XYZ + quaternion XYZW), reçu {len(msg.data)}")
            return
        
        # Extraire la position cible (x, y, z)
        target_position = np.array(msg.data[0:3])
        
        # ═══════════════════════════════════════════════════════════════
        # VÉRIFICATIONS DE SÉCURITÉ
        # ═══════════════════════════════════════════════════════════════
        
        # Vérifier les limites actuelles de la cellule
        if not self.limite_cellule():
            self.get_logger().warn("Mouvement bloqué: robot actuellement trop proche des limites de la cellule")
            return
        
        # Vérifier si la position cible est sûre
        if not self.is_target_safe(target_position):
            self.get_logger().warn("Mouvement bloqué: position cible trop proche des limites de la cellule")
            return
        
        # ═══════════════════════════════════════════════════════════════
        # TRAITEMENT DE L'ORIENTATION
        # ═══════════════════════════════════════════════════════════════
        
        # Extraire le quaternion (x, y, z, w)
        quat_x, quat_y, quat_z, quat_w = msg.data[3:7]
        
        # Normaliser le quaternion pour éviter les erreurs numériques
        quat_norm = np.sqrt(quat_x**2 + quat_y**2 + quat_z**2 + quat_w**2)
        if quat_norm < 1e-10:
            self.get_logger().error("Quaternion reçu a une norme nulle ou proche de zéro")
            return
            
        quat_normalized = np.array([quat_x, quat_y, quat_z, quat_w]) / quat_norm
        
        # Convertir le quaternion en matrice de rotation
        # Note: Pinocchio utilise la convention (w, x, y, z) pour les quaternions
        orientation_from_quat = pin.Quaternion(quat_normalized[3], quat_normalized[0], 
                                             quat_normalized[1], quat_normalized[2]).toRotationMatrix()
    
        # Appliquer une rotation supplémentaire pour l'alignement des repères
        # Rotation de π (180°) autour de l'axe X pour aligner les repères
        # robot et contrôleur haptique
        rotation_x_pi = pin.utils.rpyToMatrix(np.pi, 0, 0)
        target_orientation = orientation_from_quat @ rotation_x_pi
        
        self.get_logger().debug(f"Calcul du MGI pour pose: pos={target_position}, "
                               f"quat=[{quat_normalized[0]:.4f}, {quat_normalized[1]:.4f}, "
                               f"{quat_normalized[2]:.4f}, {quat_normalized[3]:.4f}] (avec rotation X 180°)")
        
        # ═══════════════════════════════════════════════════════════════
        # TRAITEMENT DE LA COMMANDE PINCE
        # ═══════════════════════════════════════════════════════════════
        
        # Extraire et publier la commande de pince si disponible
        if len(msg.data) >= 8:
            gripper_value = float(msg.data[7])
            self.get_logger().debug(f"Valeur de la pince reçue: {gripper_value}")
            
            # Publier la commande de pince sur le topic dédié
            gripper_msg = Float64()
            gripper_msg.data = gripper_value
            self.gripper_pub.publish(gripper_msg)
        
        # ═══════════════════════════════════════════════════════════════
        # RÉSOLUTION DE LA CINÉMATIQUE INVERSE
        # ═══════════════════════════════════════════════════════════════
        
        # Calculer la configuration articulaire via MGI
        q_solution = self.geometric_inverse(target_position, target_orientation)
        
        if q_solution is not None:
            # Vérification de la qualité de la solution par calcul du MGD
            solution_position, solution_orientation = self.geometric_direct(q_solution)
            
            # Calculer les erreurs de position et d'orientation
            position_error = np.linalg.norm(target_position - solution_position)
            
            # Pour l'erreur d'orientation, utiliser l'angle entre les deux orientations
            orientation_error_vec = pin.log3(solution_orientation.T @ target_orientation)
            orientation_error = np.linalg.norm(orientation_error_vec)
            
            self.get_logger().debug(f"Solution trouvée avec erreurs: position={position_error:.6f}m, "
                                   f"orientation={orientation_error:.6f}rad")
            
            # Publier la solution pour les contrôleurs du robot
            self.publish_joint_positions(q_solution)
            
        else:
            self.get_logger().error("Impossible de trouver une solution valide pour la pose cible")
    
    def publish_joint_positions(self, q_solution):
        """
        Publie les positions articulaires calculées vers les contrôleurs du robot.
        
        Effectue une dernière vérification de sécurité avant de publier les positions
        articulaires calculées par le solveur MGI vers les contrôleurs du robot.
        
        Args:
            q_solution (np.ndarray): Configuration articulaire calculée (7 DDL)
        """
        # Vérification finale des limites de sécurité avant publication
        if not self.limite_cellule():
            self.get_logger().warn("Publication des positions articulaires annulée: "
                                 "robot trop proche des limites de la cellule")
            return
            
        # Créer le message Float32MultiArray pour les positions articulaires
        joint_cmd = Float32MultiArray()
        
        # Convertir les positions en float32 et limiter à 7 DDL
        joint_cmd.data = [float(q_solution[i]) for i in range(min(len(q_solution), 7))]
        
        # Publier le message vers les contrôleurs
        self.joint_positions_pub.publish(joint_cmd)
    
    def geometric_direct(self, q):
        """
        Calcule le modèle géométrique direct (MGD) pour une configuration donnée.
        
        Étant donné une configuration articulaire, calcule la position et l'orientation
        de l'effecteur final dans l'espace cartésien.
        
        Args:
            q (np.ndarray): Configuration articulaire (7 DoF pour Fr3)
            
        Returns:
            tuple: (position, orientation) de l'effecteur final
                - position (np.ndarray): Position XYZ en mètres
                - orientation (np.ndarray): Matrice de rotation 3x3
        """
        # Préparer la configuration complète (ajouter positions des doigts si nécessaire)
        if q.shape[0] == 7:
            # Ajouter les positions des doigts de la pince (1 cm d'ouverture chacun)
            q = np.concatenate([q, [0.01, 0.01]])

        # Calculer la cinématique directe avec Pinocchio
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.ee_id)
        
        # Extraire la position et l'orientation de l'effecteur final
        ee_position = self.data.oMf[self.ee_id].translation
        ee_orientation = self.data.oMf[self.ee_id].rotation
        
        return ee_position, ee_orientation
    
    def geometric_inverse(self, target_position, target_orientation, q_init=None):
        """
        Calcule le modèle géométrique inverse (MGI) pour atteindre une pose cible.
        
        Résout le problème de cinématique inverse pour trouver la configuration
        articulaire permettant d'atteindre une pose cible donnée (position + orientation).
        
        Args:
            target_position (np.ndarray): Position cible XYZ en mètres
            target_orientation (np.ndarray): Matrice de rotation cible 3x3
            q_init (np.ndarray, optional): Configuration initiale pour la résolution.
                                         Si None, utilise la configuration actuelle.
        
        Returns:
            np.ndarray or None: Configuration articulaire solution (7 DoF) ou None si échec
        """
        # Utiliser la configuration actuelle comme point de départ si non spécifié
        if q_init is None:
            q_init = self.current_joint_positions.copy()
        
        # Créer la matrice de transformation homogène cible (4x4)
        target_SE3 = pin.SE3(target_orientation, target_position)
        
        # Résoudre le MGI avec la méthode de Levenberg-Marquardt
        q_result = self._solve_mgi(target_SE3, q_init)
        
        return q_result
    
    def is_configuration_valid(self, q):
        """Vérifie si une configuration est valide (pas en singularité)"""
        if q.shape[0] == 7:
            q_full = np.concatenate([q, [0.035, 0.035]])
        else:
            q_full = q.copy()
        
        pin.forwardKinematics(self.model, self.data, q_full)
        J = pin.computeFrameJacobian(self.model, self.data, q_full, self.ee_id)
        
        # Vérifier le conditionnement de la Jacobienne
        cond_number = np.linalg.cond(J)
        return cond_number < 1e6  # Seuil à ajuster
    
    def _solve_mgi(self, target_SE3, q_init):
        """
        Méthode interne pour résoudre le MGI avec l'algorithme de Levenberg-Marquardt.
        
        Implémente la résolution itérative du problème de cinématique inverse en
        utilisant la méthode de Levenberg-Marquardt avec amortissement pour éviter
        les singularités.
        
        Args:
            target_SE3 (pin.SE3): Transformation homogène cible
            q_init (np.ndarray): Configuration articulaire initiale
            
        Returns:
            np.ndarray or None: Configuration solution ou None si pas de convergence
            
        Algorithme:
            1. Calculer l'erreur entre la pose actuelle et la pose cible
            2. Calculer la Jacobienne du robot
            3. Résoudre le système linéaire avec pseudo-inverse amortie
            4. Mettre à jour la configuration et répéter jusqu'à convergence
        """
        # ═══════════════════════════════════════════════════════════════
        # PARAMÈTRES DE L'ALGORITHME
        # ═══════════════════════════════════════════════════════════════
        
        eps = self.mgi_epsilon      # Tolérance de convergence
        max_iter = self.mgi_max_iter    # Nombre maximal d'itérations
        damp = self.mgi_damping     # Facteur d'amortissement pour la pseudo-inverse
        
        # Configuration courante de la résolution
        q_result = q_init.copy()

        # ═══════════════════════════════════════════════════════════════
        # Vérifier d'abord si la configuration initiale est valide
        # ═══════════════════════════════════════════════════════════════
        if not self.is_configuration_valid(q_result):
            self.get_logger().warn("Configuration initiale proche d'une singularité")
            # Essayer de perturber légèrement la configuration
            q_result += np.random.normal(0, 0.01, q_result.shape)
        
        # ═══════════════════════════════════════════════════════════════
        # BOUCLE PRINCIPALE DE RÉSOLUTION ITÉRATIVE
        # ═══════════════════════════════════════════════════════════════
        
        for i in range(max_iter):
            # Préparer la configuration complète pour Pinocchio
            if q_result.shape[0] == 7:
                q_result = np.concatenate([q_result, [0.01, 0.01]])

            # Calculer la pose actuelle de l'effecteur (MGD)
            pin.forwardKinematics(self.model, self.data, q_result)
            pin.updateFramePlacement(self.model, self.data, self.ee_id)
            current_SE3 = self.data.oMf[self.ee_id]
            
            # Calculer l'erreur de pose (différence entre pose actuelle et cible)
            # L'erreur est exprimée dans l'espace tangent SE(3) (6D: 3 translation + 3 rotation)
            error = pin.log(current_SE3.inverse() * target_SE3).vector
            error_norm = np.linalg.norm(error)
            
            # ═══════════════════════════════════════════════════════════
            # TEST DE CONVERGENCE
            # ═══════════════════════════════════════════════════════════
            
            if error_norm < eps:
                self.get_logger().debug(f"MGI convergé après {i+1} itérations avec erreur {error_norm:.6f}")
                return q_result
            
            # ═══════════════════════════════════════════════════════════
            # CALCUL DE LA JACOBIENNE ET MISE À JOUR
            # ═══════════════════════════════════════════════════════════
            
            # Calculer la Jacobienne géométrique de l'effecteur final
            J = pin.computeFrameJacobian(self.model, self.data, q_result, self.ee_id)
            
            # Résoudre le système linéaire avec pseudo-inverse amortie (Levenberg-Marquardt)
            # Cette méthode évite les problèmes de singularité
            J_pinv = np.linalg.pinv(J, rcond=damp)
            dq = J_pinv @ error
            
            # Adaptation du pas pour éviter les changements trop brusques
            # Le facteur alpha limite la vitesse de convergence pour la stabilité
            alpha = min(1.0, 0.5 / max(1e-10, np.linalg.norm(dq)))
            if error_norm > self.prev_error_norm:  # Si l'erreur augmente
                alpha *= 0.5  # Réduire le pas
            else:
                alpha = min(1.0, alpha * 1.1)  # Augmenter légèrement le pas

            q_result += alpha * dq
            self.prev_error_norm = error_norm
            
            # Normaliser les angles et appliquer les contraintes articulaires
            q_result = pin.normalize(self.model, q_result)
            
            # Appliquer les limites articulaires physiques du robot si disponibles
            if hasattr(self.model, 'lowerPositionLimit') and hasattr(self.model, 'upperPositionLimit'):
                q_result = np.minimum(np.maximum(q_result, self.model.lowerPositionLimit), 
                                     self.model.upperPositionLimit)
        
        # ═══════════════════════════════════════════════════════════════
        # GESTION DE LA NON-CONVERGENCE
        # ═══════════════════════════════════════════════════════════════
        
        # Si nous atteignons le nombre maximal d'itérations sans converger
        self.get_logger().warn(f"MGI n'a pas convergé après {max_iter} itérations. "
                              f"Erreur finale: {error_norm:.6f}")
        
        # Vérifier si la solution est "assez bonne" malgré la non-convergence
        if error_norm < 10*eps:  # Tolérance élargie pour les solutions approximatives
            self.get_logger().debug(f"Solution approximative trouvée avec erreur {error_norm:.6f}")
            return q_result
        
        # Échec total de la convergence
        return None


def main(args=None):
    """
    Fonction principale pour lancer le nœud ROS2.
    
    Initialise le système ROS2, créé une instance du nœud FrankaMGSolver,
    et gère proprement l'arrêt du nœud.
    
    Args:
        args: Arguments de ligne de commande (optionnel)
    """
    # Initialisation du système ROS2
    rclpy.init(args=args)
    
    # Création de l'instance du nœud
    franka_solver = FrankaMGSolver()
    
    try:
        # Boucle principale ROS2 - traitement des callbacks
        rclpy.spin(franka_solver)
    except KeyboardInterrupt:
        # Arrêt propre sur Ctrl+C
        pass
    finally:
        # Nettoyage des ressources
        franka_solver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()