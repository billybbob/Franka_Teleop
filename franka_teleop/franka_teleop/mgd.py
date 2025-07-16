#!/usr/bin/env python3
"""
Node ROS2 pour calculer le Modèle Géométrique Direct (MGD) du Franka Fr3
en utilisant la bibliothèque Pinocchio.

Ce module fait partie d'un projet de téléopération d'un robot Franka Fr3 
avec un contrôleur haptique Desktop 6D de Haption. Il calcule en temps réel
la position et l'orientation de l'effecteur terminal du robot à partir des
positions articulaires reçues via les topics ROS2.

Le MGD (Modèle Géométrique Direct) permet de déterminer la pose de l'effecteur
terminal en fonction des angles des articulations du robot. Cette information
est essentielle pour la téléopération car elle permet de synchroniser les
mouvements du robot avec le retour haptique.

Auteur: Vincent Bassemayousse
Date: 07/10/2025
Version: 1.0
Licence: Apache 2.0
Environnement: ROS2 Humble, Gazebo Fortress
"""

# ============================================================================
# IMPORTATION DES DIFFÉRENTS ÉLÉMENTS UTILES
# ============================================================================

import numpy as np                      # Calculs numériques et matrices
import pinocchio as pin                 # Bibliothèque de robotique pour MGD/MGI
import rclpy                           # Client ROS2 Python
from rclpy.node import Node            # Classe de base pour les nodes ROS2
from sensor_msgs.msg import JointState # Messages d'état des articulations
from geometry_msgs.msg import TransformStamped  # Messages de transformation
from tf2_msgs.msg import TFMessage     # Messages TF pour les transformations
import os                              # Opérations sur le système de fichiers
import subprocess                      # Exécution de commandes système
from ament_index_python.packages import get_package_share_directory  # Localisation des packages ROS2

# ============================================================================
# CLASSE PRINCIPALE DU NODE MGD
# ============================================================================

class MGDNode(Node):
    """
    Node ROS2 pour calculer le Modèle Géométrique Direct du robot Franka Fr3
    en utilisant la bibliothèque Pinocchio.
    
    Cette classe hérite de Node (rclpy) et implémente un système de calcul
    en temps réel de la position et orientation de l'effecteur terminal du
    robot Franka Fr3. Elle utilise Pinocchio pour les calculs de cinématique
    directe à partir du modèle URDF du robot.
    
    Le node subscribe aux états des articulations et publie la transformation
    de l'effecteur terminal, permettant ainsi aux autres composants du système
    de téléopération (notamment le contrôleur haptique) de connaître la pose
    actuelle du robot.
    
    Attributes:
        r1, r3, r5 (float): Longueurs des segments du robot (en mètres)
        d4, d5 (float): Offsets des articulations (en mètres)
        q (np.array): Vecteur des positions articulaires actuelles (en radians)
        dt (float): Période d'échantillonnage pour les calculs (en secondes)
        model (pin.Model): Modèle Pinocchio du robot
        data (pin.Data): Données de calcul associées au modèle
        ee_id (int): Identifiant du frame de l'effecteur terminal
        
    Publishers:
        /position_effecteur_robot (TFMessage): Position et orientation de l'effecteur
        
    Subscribers:
        /NS_1/joint_states (JointState): États des articulations du robot
    """
    
    def __init__(self):
        """
        Constructeur du node MGD.
        
        Initialise tous les paramètres du robot, configure le modèle Pinocchio,
        crée les publishers/subscribers ROS2 et démarre le timer de calcul
        périodique du MGD.
        
        Les paramètres géométriques du robot sont basés sur les spécifications
        officielles du Franka Fr3. La configuration articulaire initiale
        correspond à une pose de "ready" typique du robot.
        """
        super().__init__('mgd_node')
        
        # ====================================================================
        # PARAMÈTRES GÉOMÉTRIQUES DU ROBOT FRANKA FR3
        # ====================================================================
        # Ces valeurs correspondent aux dimensions physiques du robot
        # et sont utilisées pour valider les calculs Pinocchio si nécessaire
        self.r1 = 0.333    # Hauteur de la base au joint 2 (en mètres)
        self.r3 = 0.316    # Longueur du segment joint 2 - joint 4 (en mètres)
        self.r5 = 0.384    # Longueur du segment joint 4 - joint 6 (en mètres)
        self.d4 = 0.0825   # Offset du joint 4 (en mètres)
        self.d5 = -0.0825  # Offset du joint 5 (en mètres)
        
        # ====================================================================
        # CONFIGURATION ARTICULAIRE INITIALE
        # ====================================================================
        # Position initiale du robot en configuration "ready"
        # Les 7 valeurs correspondent aux 7 articulations du Franka Fr3
        # [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
        self.q = np.array([
            -2.8737350679638207e-05,  # Joint 1: rotation base (≈ 0°)
            -0.785398157365465,       # Joint 2: épaule (≈ -45°)
            3.592721522664258e-05,    # Joint 3: bras (≈ 0°)
            -2.356194490192345,       # Joint 4: coude (≈ -135°)
            2.4004978253540097e-05,   # Joint 5: avant-bras (≈ 0°)
            1.5707963267948963,       # Joint 6: poignet 1 (≈ 90°)
            0.7853981634216485        # Joint 7: poignet 2 (≈ 45°)
        ])

        # ====================================================================
        # PARAMÈTRES DE CONTRÔLE TEMPOREL
        # ====================================================================
        # Fréquence de calcul du MGD - 200 Hz pour un retour temps réel
        # Cette fréquence élevée est nécessaire pour la téléopération haptique
        self.dt = 0.005  # Période = 5ms -> Fréquence = 200 Hz

        # ====================================================================
        # INITIALISATION DU MODÈLE PINOCCHIO
        # ====================================================================
        # Chargement et configuration du modèle cinématique du robot
        self.setup_pinocchio_model()

        # ====================================================================
        # CONFIGURATION DES COMMUNICATIONS ROS2
        # ====================================================================
        
        # Subscriber pour recevoir les états des articulations du robot
        # Le topic '/NS_1/joint_states' est publié par Gazebo ou le driver du robot réel
        self.joint_sub = self.create_subscription(
            JointState,                     # Type de message
            '/NS_1/joint_states',           # Pour le robot réel
            # '/joint_states',              # Pour la simulation sous Gazebo
            self.joint_callback,            # Fonction de callback
            10                              # Taille de la queue (buffer)
        )
        
        # Publisher pour envoyer la position de l'effecteur terminal
        # Ce topic est utilisé par les autres nodes du système de téléopération
        self.position_effecteur_robot_pub = self.create_publisher(
            TFMessage,                     # Type de message (transformation)
            '/position_effecteur_robot',   # Nom du topic
            10                            # Taille de la queue
        )

        # ====================================================================
        # TIMER POUR LE CALCUL PÉRIODIQUE
        # ====================================================================
        # Timer qui déclenche le calcul et la publication de la position
        # de l'effecteur à la fréquence définie (200 Hz)
        self.position_timer = self.create_timer(
            self.dt,                           # Période du timer
            self.calculate_and_publish_position # Fonction appelée périodiquement
        )

        # ====================================================================
        # MESSAGES DE DÉMARRAGE
        # ====================================================================
        self.get_logger().info('Node MGD démarré avec succès')
        self.get_logger().info(f'Fréquence de calcul: {1/self.dt:.1f} Hz')
        self.get_logger().info(f'Positions articulaires initiales: {self.q}')

    def load_urdf_from_xacro(self):
        """
        Charge le modèle URDF du robot à partir du fichier XACRO.
        
        Le fichier XACRO (XML Macro) est un format paramétrable qui permet
        de définir des robots de manière modulaire. Cette fonction localise
        le fichier XACRO du Franka Fr3, le convertit en URDF standard,
        et retourne le contenu XML résultant.
        
        Le processus implique:
        1. Localisation du package franka_description
        2. Construction du chemin vers le fichier XACRO
        3. Conversion XACRO -> URDF via l'outil en ligne de commande
        4. Validation et retour du contenu URDF
        
        Returns:
            str: Contenu du fichier URDF généré à partir du XACRO
            
        Raises:
            FileNotFoundError: Si le fichier XACRO n'est pas trouvé
            RuntimeError: Si la conversion XACRO échoue
            
        Note:
            Cette méthode nécessite que le package franka_description soit
            installé et que l'outil 'xacro' soit disponible dans le PATH.
        """
        # Localisation du package ROS2 contenant les descriptions du robot
        try:
            franka_desc_path = get_package_share_directory('franka_description')
        except Exception as e:
            self.get_logger().error(f"Package franka_description non trouvé: {e}")
            raise
            
        # Construction du chemin complet vers le fichier XACRO du Fr3
        xacro_path = os.path.join(franka_desc_path, 'robots', 'fr3', 'fr3.urdf.xacro')
        
        self.get_logger().info(f"Chargement du modèle XACRO: {xacro_path}")
        
        # Vérification de l'existence du fichier
        if not os.path.exists(xacro_path):
            error_msg = f"Fichier XACRO introuvable: {xacro_path}"
            self.get_logger().error(error_msg)
            raise FileNotFoundError(error_msg)
        
        # Conversion XACRO en URDF en utilisant l'outil en ligne de commande
        # La commande 'xacro' traite les macros et génère un URDF standard
        try:
            result = subprocess.run(
                ['xacro', xacro_path],        # Commande et arguments
                stdout=subprocess.PIPE,        # Capture de la sortie standard
                stderr=subprocess.PIPE,        # Capture des erreurs
                text=True,                     # Mode texte (pas binaire)
                timeout=30                     # Timeout de 30 secondes
            )
        except subprocess.TimeoutExpired:
            error_msg = "Timeout lors de la conversion XACRO (>30s)"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
        except FileNotFoundError:
            error_msg = "Outil 'xacro' non trouvé. Installez ros-humble-xacro"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
        
        # Vérification du succès de la conversion
        if result.returncode != 0:
            error_msg = f"Erreur lors de la conversion XACRO: {result.stderr}"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
        
        # Validation basique du contenu URDF généré
        urdf_content = result.stdout
        if not urdf_content.strip():
            error_msg = "Le fichier URDF généré est vide"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
            
        if '<robot' not in urdf_content:
            error_msg = "Le contenu généré ne semble pas être un URDF valide"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
        
        self.get_logger().info("Modèle URDF chargé avec succès")
        return urdf_content
        
    def setup_pinocchio_model(self):
        """
        Configure le modèle Pinocchio pour le robot Franka Fr3.
        
        Cette méthode initialise le modèle cinématique utilisé pour les calculs
        de MGD (Modèle Géométrique Direct). Elle effectue les étapes suivantes:
        
        1. Chargement du modèle URDF depuis le fichier XACRO
        2. Construction du modèle Pinocchio à partir de l'URDF
        3. Création des structures de données associées
        4. Identification du frame de l'effecteur terminal
        5. Validation de la configuration
        
        Le modèle Pinocchio permet d'effectuer efficacement les calculs de
        cinématique directe et inverse, essentiels pour la téléopération.
        
        Raises:
            Exception: Si le chargement du modèle échoue
            
        Note:
            Cette méthode tente de trouver automatiquement le frame de
            l'effecteur terminal parmi plusieurs noms possibles. En cas
            d'échec, elle utilise le dernier frame du modèle.
        """
        try:
            self.get_logger().info("Initialisation du modèle Pinocchio...")
            
            # ================================================================
            # CHARGEMENT DU MODÈLE URDF
            # ================================================================
            # Conversion XACRO -> URDF et chargement du contenu
            urdf_string = self.load_urdf_from_xacro()
            
            # ================================================================
            # CONSTRUCTION DU MODÈLE PINOCCHIO
            # ================================================================
            # Création du modèle cinématique à partir de la description URDF
            self.model = pin.buildModelFromXML(urdf_string)
            
            # Création des structures de données pour les calculs
            # (contient les résultats des calculs de cinématique)
            self.data = self.model.createData()
            
            # ================================================================
            # IDENTIFICATION DE L'EFFECTEUR TERMINAL
            # ================================================================
            # Le frame de l'effecteur peut avoir différents noms selon la version
            # du modèle et la configuration utilisée
            potential_ee_frames = [
                'fr3_finger_joint1',    # Frame par défaut pour les pinces
                'fr3_link8',           # Dernier lien du bras
                'fr3_hand_tcp',        # Tool Center Point de la main
                'fr3_hand',            # Frame de la main
                'panda_link8',         # Nom alternatif (compatibilité Panda)
                'panda_hand',          # Main Panda (rétrocompatibilité)
                'panda_hand_tcp'       # TCP Panda
            ]
            
            # Recherche du frame de l'effecteur terminal
            frame_found = False
            for frame_name in potential_ee_frames:
                if self.model.existFrame(frame_name):
                    self.ee_id = self.model.getFrameId(frame_name)
                    self.get_logger().info(f'Frame effecteur trouvé: {frame_name} (ID: {self.ee_id})')
                    frame_found = True
                    break
            
            # Fallback si aucun frame connu n'est trouvé
            if not frame_found:
                # Utilisation du dernier frame comme approximation
                self.ee_id = self.model.nframes - 1
                ee_frame_name = self.model.frames[self.ee_id].name
                self.get_logger().warning(
                    f'Aucun frame d\'effecteur standard trouvé. '
                    f'Utilisation du frame: {ee_frame_name} (ID: {self.ee_id})'
                )
            
            # ================================================================
            # VALIDATION ET INFORMATION SUR LE MODÈLE
            # ================================================================
            self.get_logger().info(f'Modèle Pinocchio configuré avec succès:')
            self.get_logger().info(f'  - Nombre d\'articulations (nq): {self.model.nq}')
            self.get_logger().info(f'  - Nombre de vitesses (nv): {self.model.nv}')
            self.get_logger().info(f'  - Nombre de frames: {self.model.nframes}')
            self.get_logger().info(f'  - Frame effecteur: ID {self.ee_id}')
            
            # Vérification de cohérence basique
            if self.model.nq < 7:
                self.get_logger().warning(
                    f'Le modèle a moins de 7 articulations ({self.model.nq}). '
                    'Vérifiez que le modèle Franka Fr3 est correct.'
                )
            
        except Exception as e:
            error_msg = f'Erreur lors de la configuration du modèle Pinocchio: {str(e)}'
            self.get_logger().error(error_msg)
            self.get_logger().error('Le node ne peut pas fonctionner sans modèle valide.')
            raise

    def joint_callback(self, msg):
        """
        Callback pour traiter les messages d'état des articulations.
        
        Cette fonction est appelée automatiquement chaque fois qu'un nouveau
        message JointState est reçu sur le topic '/NS_1/joint_states'. Elle
        met à jour l'état interne des positions articulaires du robot.
        
        Le message JointState contient:
        - name: Liste des noms des articulations
        - position: Liste des positions correspondantes (en radians)
        - velocity: Vitesses (optionnel, non utilisé ici)
        - effort: Couples/forces (optionnel, non utilisé ici)
        
        Args:
            msg (sensor_msgs.msg.JointState): Message contenant l'état des articulations
            
        Note:
            Cette fonction garantit que les positions sont assignées dans le bon
            ordre, indépendamment de l'ordre dans le message reçu. Elle gère
            aussi les cas où certaines articulations pourraient être manquantes.
        """
        # ====================================================================
        # EXTRACTION DES DONNÉES DU MESSAGE
        # ====================================================================
        # Création d'un dictionnaire nom_articulation -> position
        # pour faciliter l'accès aux données
        joint_dict = dict(zip(msg.name, msg.position))
        
        # ====================================================================
        # ORDRE CANONIQUE DES ARTICULATIONS FRANKA FR3
        # ====================================================================
        # Définition de l'ordre attendu des 7 articulations principales
        # (les articulations des pinces sont exclues car non nécessaires pour le MGD)
        joint_order = [
            'fr3_joint1',    # Base - rotation autour de l'axe Z
            'fr3_joint2',    # Épaule - rotation dans le plan sagittal
            'fr3_joint3',    # Bras - rotation autour de l'axe du bras
            'fr3_joint4',    # Coude - flexion/extension
            'fr3_joint5',    # Avant-bras - rotation autour de l'axe de l'avant-bras
            'fr3_joint6',    # Poignet 1 - flexion/extension du poignet
            'fr3_joint7'     # Poignet 2 - rotation finale de l'effecteur
        ]
        
        # ====================================================================
        # MISE À JOUR DES POSITIONS ARTICULAIRES
        # ====================================================================
        # Compteur pour vérifier combien d'articulations ont été mises à jour
        updated_joints = 0
        
        for i, joint_name in enumerate(joint_order):
            if joint_name in joint_dict:
                # Mise à jour de la position de l'articulation i
                old_value = self.q[i]
                self.q[i] = joint_dict[joint_name]
                updated_joints += 1
                
                # Log des changements significatifs (> 0.01 rad ≈ 0.57°)
                if abs(self.q[i] - old_value) > 0.01:
                    self.get_logger().debug(
                        f'{joint_name}: {old_value:.4f} -> {self.q[i]:.4f} rad '
                        f'(Δ = {self.q[i] - old_value:.4f})'
                    )
            else:
                # Articulation manquante dans le message
                self.get_logger().warning(
                    f'Articulation manquante dans le message: {joint_name}'
                )
        
        # ====================================================================
        # VALIDATION ET LOGGING
        # ====================================================================
        if updated_joints != len(joint_order):
            self.get_logger().warning(
                f'Seulement {updated_joints}/{len(joint_order)} articulations mises à jour'
            )
        
        # Log périodique de l'état complet (toutes les 100 réceptions ≈ 0.5s à 200Hz)
        if not hasattr(self, '_callback_count'):
            self._callback_count = 0
        self._callback_count += 1
        
        if self._callback_count % 100 == 0:
            self.get_logger().debug(f'État articulaire complet: {self.q}')
            # Conversion en degrés pour une lecture plus intuitive
            q_degrees = np.degrees(self.q)
            self.get_logger().debug(f'État articulaire (degrés): {q_degrees}')

    def calculate_and_publish_position(self):
        """Calculer et publier la position de l'effecteur terminal"""
        try:
            # Calcul du MGD avec Pinocchio
            position, orientation = self.calcul_MGD(self.q)
            
            # Créer la transformation
            transform = self.create_transform_stamped(position, orientation)
            
            # Créer et publier le message TF
            tf_msg = TFMessage(transforms=[transform])
            self.position_effecteur_robot_pub.publish(tf_msg)
            
        except Exception as e:
            self.get_logger().error(f'Erreur dans le calcul du MGD: {str(e)}')

    def calcul_MGD(self, q):
        """
        Calcule le modèle géométrique direct pour une configuration articulaire donnée.
        
        Cette méthode implémente le calcul de la cinématique directe en utilisant
        Pinocchio. Elle transforme les angles des articulations en position et
        orientation 3D de l'effecteur terminal dans le repère de base du robot.
        
        Le processus de calcul comprend:
        1. Adaptation du vecteur d'articulations au modèle complet
        2. Calcul de la cinématique directe avec Pinocchio
        3. Mise à jour des transformations des frames
        4. Extraction de la pose de l'effecteur terminal
        
        Args:
            q (np.array): Vecteur des angles des articulations (radians)
                         Doit contenir au moins 7 éléments pour le Fr3
        
        Returns:
            tuple: (position_effecteur, orientation_effecteur)
                - position_effecteur (np.array): Position [x, y, z] en mètres
                - orientation_effecteur (np.array): Matrice de rotation 3x3
        
        Raises:
            ValueError: Si le vecteur q n'a pas la bonne dimension
            RuntimeError: Si le calcul Pinocchio échoue
            
        Note:
            La méthode gère automatiquement les modèles avec pinces (9 DOF)
            en ajoutant des valeurs par défaut pour les articulations des doigts.
        """
        # ====================================================================
        # VALIDATION DES ENTRÉES
        # ====================================================================
        if len(q) < 7:
            raise ValueError(f"Le vecteur q doit avoir au moins 7 éléments, reçu: {len(q)}")
        
        # ====================================================================
        # ADAPTATION AU MODÈLE COMPLET
        # ====================================================================
        # Le modèle Pinocchio peut inclure plus d'articulations que les 7 principales
        # (par exemple, les 2 articulations des pinces pour un total de 9)
        q_full = np.zeros(self.model.nq)
        
        # Copie des articulations principales (au maximum 7 ou nq si plus petit)
        n_joints_to_copy = min(len(q), self.model.nq, 7)
        q_full[:n_joints_to_copy] = q[:n_joints_to_copy]
            
        # ====================================================================
        # GESTION DES ARTICULATIONS SUPPLÉMENTAIRES (PINCES)
        # ====================================================================
        if self.model.nq > 7:
            # Si le modèle inclut les pinces, on définit une ouverture par défaut
            # Valeur typique pour une pince légèrement ouverte (1 cm)
            finger_opening = 0.01  # en mètres
            
            # Les 2 dernières articulations sont généralement les doigts
            # fr3_finger_joint1 et fr3_finger_joint2
            if self.model.nq == 9:  # 7 joints du bras + 2 doigts
                q_full[7] = finger_opening   # Doigt 1
                q_full[8] = finger_opening   # Doigt 2
            else:
                # Pour d'autres configurations, on met les articulations restantes à 0
                q_full[7:] = 0.0
        
        try:
            # ================================================================
            # CALCUL DE LA CINÉMATIQUE DIRECTE
            # ================================================================
            # Calcul des positions et orientations de tous les liens du robot
            pin.forwardKinematics(self.model, self.data, q_full)
            
            # ================================================================
            # MISE À JOUR DES FRAMES
            # ================================================================
            # Calcul des transformations de tous les frames (incluant l'effecteur)
            pin.updateFramePlacement(self.model, self.data, self.ee_id)
            
            # ================================================================
            # EXTRACTION DE LA POSE DE L'EFFECTEUR
            # ================================================================
            # Récupération de la transformation homogène de l'effecteur
            # oMf[i] = transformation du frame i par rapport au repère monde
            ee_transform = self.data.oMf[self.ee_id]
            
            # Extraction de la position (vecteur 3D)
            position_effecteur = ee_transform.translation.copy()
            
            # Extraction de l'orientation (matrice 3x3)
            orientation_effecteur = ee_transform.rotation.copy()
            
            # ================================================================
            # VALIDATION DES RÉSULTATS
            # ================================================================
            # Vérifications basiques de cohérence
            if np.any(np.isnan(position_effecteur)):
                raise RuntimeError("Position de l'effecteur contient des NaN")
            
            if np.any(np.isnan(orientation_effecteur)):
                raise RuntimeError("Orientation de l'effecteur contient des NaN")
            
            # Vérification que la matrice de rotation est orthogonale
            # (det(R) ≈ 1 et R^T * R ≈ I)
            det_rot = np.linalg.det(orientation_effecteur)
            if abs(det_rot - 1.0) > 1e-6:
                self.get_logger().warning(
                    f"Matrice de rotation non-orthogonale (det={det_rot:.6f})"
                )
            
            # Vérification de l'espace de travail du robot (limites physiques)
            position_norm = np.linalg.norm(position_effecteur)
            if position_norm > 2.0:  # Portée maximale approximative du Fr3
                self.get_logger().warning(
                    f"Position effecteur hors de l'espace de travail attendu: "
                    f"distance={position_norm:.3f}m"
                )
            
            return position_effecteur, orientation_effecteur
            
        except Exception as e:
            error_msg = f"Erreur dans le calcul Pinocchio: {str(e)}"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)

    def create_transform_stamped(self, position, orientation, 
                                 child_frame='end_effector', parent_frame='base'):
        """
        Crée un message TransformStamped à partir d'une position et d'une orientation.
        
        Cette méthode convertit les résultats du calcul MGD (position 3D et matrice
        de rotation) en un message ROS2 standard TransformStamped. Ce format est
        utilisé par le système TF2 de ROS pour représenter les transformations
        entre repères.
        
        Le message créé contient:
        - Header avec timestamp et frame de référence
        - Translation (position 3D)
        - Rotation (quaternion)
        - Identification des frames parent et enfant
        
        Args:
            position (np.array): Position 3D [x, y, z] en mètres
            orientation (np.array): Matrice de rotation 3x3
            child_frame (str, optional): Nom du frame enfant (effecteur). 
                                       Par défaut 'end_effector'
            parent_frame (str, optional): Nom du frame parent (base robot). 
                                        Par défaut 'base'
            
        Returns:
            TransformStamped: Message de transformation ROS2 prêt à être publié
            
        Raises:
            ImportError: Si scipy n'est pas disponible pour la conversion quaternion
            ValueError: Si la matrice de rotation n'est pas valide
            
        Note:
            La conversion matrice de rotation -> quaternion utilise scipy.spatial.transform
            qui garantit une conversion numériquement stable et normalisée.
        """
        # ====================================================================
        # CRÉATION DU MESSAGE DE BASE
        # ====================================================================
        t = TransformStamped()
        
        # ====================================================================
        # CONFIGURATION DE L'HEADER
        # ====================================================================
        # Timestamp actuel pour synchronisation temporelle
        t.header.stamp = self.get_clock().now().to_msg()
        
        # Frame de référence (généralement la base du robot)
        t.header.frame_id = parent_frame
        
        # Frame cible (effecteur terminal)
        t.child_frame_id = child_frame

        # ====================================================================
        # CONFIGURATION DE LA TRANSLATION
        # ====================================================================
        # Conversion numpy array -> float ROS2 (évite les problèmes de type)
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])

        # ====================================================================
        # CONVERSION ET CONFIGURATION DE LA ROTATION
        # ====================================================================
        try:
            # Import de scipy pour la conversion de rotation
            from scipy.spatial.transform import Rotation as R
            
            # Validation de la matrice de rotation
            if orientation.shape != (3, 3):
                raise ValueError(f"Matrice de rotation invalide: shape {orientation.shape}")
            
            # Conversion matrice de rotation -> objet Rotation scipy
            rot = R.from_matrix(orientation)
            
            # Extraction du quaternion [x, y, z, w] (convention scipy)
            quat = rot.as_quat()
            
            # Vérification de la normalisation du quaternion
            quat_norm = np.linalg.norm(quat)
            if abs(quat_norm - 1.0) > 1e-6:
                self.get_logger().warning(
                    f"Quaternion non-normalisé (norme={quat_norm:.6f})"
                )
                # Renormalisation si nécessaire
                quat = quat / quat_norm
            
            # Assignation au message ROS2 (ordre ROS: x, y, z, w)
            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])
            
        except ImportError:
            error_msg = "scipy.spatial.transform non disponible pour conversion quaternion"
            self.get_logger().error(error_msg)
            raise ImportError(error_msg)
        except Exception as e:
            error_msg = f"Erreur lors de la conversion rotation->quaternion: {str(e)}"
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)

        # ====================================================================
        # LOGGING DE DEBUG (PÉRIODIQUE)
        # ====================================================================
        # Log détaillé toutes les 2000 créations (≈ 10s à 200Hz)
        if not hasattr(self, '_transform_count'):
            self._transform_count = 0
        self._transform_count += 1
        
        if self._transform_count % 2000 == 0:
            self.get_logger().debug(
                f"Transform créé: {parent_frame} -> {child_frame}\n"
                f"  Position: [{position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}]\n"
                f"  Quaternion: [{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]"
            )

        return t

# ============================================================================
# FONCTION PRINCIPALE
# ============================================================================
    
def main(args=None):
    """
    Point d'entrée principal du node MGD.
    
    Cette fonction initialise ROS2, crée une instance du node MGD,
    et gère le cycle de vie complet du programme. Elle inclut la
    gestion des erreurs et un arrêt propre du node.
    
    Le cycle de vie comprend:
    1. Initialisation de ROS2
    2. Création et configuration du node MGD
    3. Boucle principale (spin) pour traitement des messages
    4. Gestion des interruptions utilisateur (Ctrl+C)
    5. Nettoyage et arrêt propre
    
    Args:
        args (list, optional): Arguments en ligne de commande. 
                              Par défaut None (utilise sys.argv)
    
    Note:
        Cette fonction est conçue pour être robuste et maintenir
        le système en fonctionnement même en cas d'erreurs non-critiques.
        Elle assure toujours un arrêt propre des ressources ROS2.
    """
    # ========================================================================
    # INITIALISATION DE ROS2
    # ========================================================================
    try:
        # Initialisation du système ROS2 avec les arguments fournis
        rclpy.init(args=args)
        print("ROS2 initialisé avec succès")
        
    except Exception as e:
        print(f"Erreur lors de l'initialisation ROS2: {e}")
        return 1  # Code d'erreur
    
    # ========================================================================
    # CRÉATION ET CONFIGURATION DU NODE
    # ========================================================================
    mgd_node = None
    try:
        # Instanciation du node MGD
        # (le constructeur configure automatiquement tous les composants)
        mgd_node = MGDNode()
        print("Node MGD créé et configuré avec succès")
        
        # Message de démarrage pour l'utilisateur
        mgd_node.get_logger().info("=" * 60)
        mgd_node.get_logger().info("NODE MGD FRANKA FR3 - TÉLÉOPÉRATION HAPTIQUE")
        mgd_node.get_logger().info("Système prêt pour calcul temps réel du MGD")
        mgd_node.get_logger().info("Appuyez sur Ctrl+C pour arrêter proprement")
        mgd_node.get_logger().info("=" * 60)
        
    except Exception as e:
        print(f"Erreur lors de la création du node MGD: {e}")
        if mgd_node is not None:
            mgd_node.destroy_node()
        rclpy.shutdown()
        return 1
    
    # ========================================================================
    # BOUCLE PRINCIPALE DU NODE
    # ========================================================================
    try:
        # Boucle infinie de traitement des messages ROS2
        # spin() bloque jusqu'à ce que le node soit interrompu
        rclpy.spin(mgd_node)
        
    except KeyboardInterrupt:
        # Gestion de l'interruption utilisateur (Ctrl+C)
        mgd_node.get_logger().info("\n" + "=" * 50)
        mgd_node.get_logger().info("ARRÊT DEMANDÉ PAR L'UTILISATEUR")
        mgd_node.get_logger().info("Arrêt en cours...")
        
    except Exception as e:
        # Gestion des erreurs critiques pendant l'exécution
        mgd_node.get_logger().error(f"Erreur critique dans le node MGD: {str(e)}")
        mgd_node.get_logger().error("Le node va s'arrêter")
        
    # ========================================================================
    # NETTOYAGE ET ARRÊT PROPRE
    # ========================================================================
    finally:
        try:
            if mgd_node is not None:
                # Statistiques finales avant arrêt
                if hasattr(mgd_node, '_calc_count'):
                    total_calcs = mgd_node._calc_count
                    uptime = total_calcs * mgd_node.dt  # Temps de fonctionnement approximatif
                    mgd_node.get_logger().info(f"Statistiques de fonctionnement:")
                    mgd_node.get_logger().info(f"  - Calculs MGD effectués: {total_calcs}")
                    mgd_node.get_logger().info(f"  - Temps de fonctionnement: {uptime:.1f}s")
                    mgd_node.get_logger().info(f"  - Fréquence moyenne: {total_calcs/uptime:.1f} Hz")
                
                # Destruction propre du node
                mgd_node.destroy_node()
                mgd_node.get_logger().info("Node détruit proprement")
                
        except Exception as e:
            print(f"Erreur lors du nettoyage: {e}")
        
        finally:
            # Arrêt du système ROS2
            try:
                rclpy.shutdown()
                print("ROS2 arrêté proprement")
            except Exception as e:
                print(f"Erreur lors de l'arrêt ROS2: {e}")
    
    # ========================================================================
    # CODE DE RETOUR
    # ========================================================================
    print("Arrêt complet du node MGD")
    return 0  # Succès

# ============================================================================
# POINT D'ENTRÉE DU SCRIPT
# ============================================================================

if __name__ == '__main__':
    """
    Point d'entrée du script Python.
    
    Cette section est exécutée seulement si le fichier est lancé directement
    (pas importé comme module). Elle appelle la fonction main() et gère
    le code de retour du programme.
    
    Usage:
        python3 mgd.py                    # Lancement direct
        ros2 run <package_name> mgd.py    # Lancement via ROS2
    """
    import sys
    
    try:
        # Lancement de la fonction principale
        exit_code = main()
        
        # Sortie avec le code de retour approprié
        sys.exit(exit_code)
        
    except Exception as e:
        # Gestion des erreurs non-capturées
        print(f"Erreur fatale non-gérée: {e}")
        sys.exit(1)