#!/usr/bin/env python3
"""
Module de Guide Virtuel pour Téléopération Robotique
===================================================

Ce module implémente un système de guide virtuel pour la téléopération d'un robot Franka FR3
avec un contrôleur haptique Desktop 6D de Haption. Le système fournit un retour de force
pour guider l'opérateur vers des objets cibles dans l'environnement de simulation Gazebo Fortress.

Auteur: Vincent Bassemayousse
Date: 07/10/2025
Version: 1.0
Licence: Apache 2.0

Architecture:
    - Nœud ROS2 pour la gestion du guide virtuel
    - Calcul de forces basé sur la distance robot-objet
    - Amortissement proportionnel à la vitesse du contrôleur
    - Modes de fonctionnement: Position avec/sans guide, Vitesse

Topics ROS2:
    Publishers:
        - /force_guide (Float32MultiArray): Forces de retour haptique [fx,fy,fz,rx,ry,rz]
    
    Subscribers:
        - /valeur_effecteur (Float32MultiArray): Pose du contrôleur [x,y,z,qx,qy,qz,qw]
        - /Mode_Pose_Vitesse (Float32MultiArray): Mode de contrôle [pose_flag,vel_flag,x,y,guide_flag]
        - /distance_robot_objet (Float32MultiArray): Distances [dist_totale,dx,dy,dz]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import math


class GuideVirtuel(Node):
    """
    Nœud ROS2 pour la création et gestion d'un guide virtuel haptique.
    
    Cette classe implémente un système de guide virtuel qui calcule des forces de retour
    haptique pour guider l'opérateur vers des objets cibles. Le système combine:
    - Forces de guidage proportionnelles à la distance
    - Forces d'amortissement proportionnelles à la vitesse
    - Gestion des modes de contrôle (position/vitesse)
    
    Attributes:
        controleur_x, controleur_y, controleur_z (float): Position cartésienne du contrôleur [m]
        controleur_qx, controleur_qy, controleur_qz, controleur_qw (float): Orientation quaternion
        force_x, force_y, force_z (float): Forces de translation calculées [N]
        force_rx, force_ry, force_rz (float): Couples de rotation calculés [N⋅m]
        distance_totale (float): Distance euclidienne robot-objet [m]
        distance_x, distance_y, distance_z (float): Distances par axe [m]
        current_velocity (list): Vitesses instantanées [vx, vy, vz] [m/s]
        in_pose_mode (bool): True si en mode position, False si en mode vitesse
        guide_active (bool): True si le guide virtuel est activé
    """
    
    def __init__(self):
        """
        Initialise le nœud de guide virtuel.
        
        Configure tous les publishers, subscribers, variables d'état et paramètres
        nécessaires au fonctionnement du guide virtuel haptique.
        """
        super().__init__('guide_virtuel')

        # ========================================================================================
        # CONFIGURATION DES TOPICS ROS2
        # ========================================================================================
        
        # Publisher pour envoyer les forces de retour haptique au contrôleur
        self.guide_force_pub = self.create_publisher(
            Float32MultiArray,
            '/force_guide',
            10
        )
        
        # Subscriber pour recevoir la pose (position + orientation) du contrôleur haptique
        self.position_controleur_sub = self.create_subscription(
            Float32MultiArray, 
            '/valeur_effecteur',  # Topic contenant [x, y, z, qx, qy, qz, qw]
            self.position_controleur_sub_callback, 
            10
        )

        # Subscriber pour recevoir le mode de contrôle actuel
        self.mode_sub = self.create_subscription(
            Float32MultiArray,
            '/Mode_Pose_Vitesse',  # Topic contenant [pose_flag, vel_flag, x, y, guide_flag]
            self.mode_callback,
            10
        )

        # Subscriber pour recevoir les distances entre le robot et les objets cibles
        self.distance_sub = self.create_subscription(
            Float32MultiArray,
            '/distance_robot_objet',  # Topic contenant [dist_totale, dx, dy, dz]
            self.distance_callback,
            10
        )

        # ========================================================================================
        # VARIABLES D'ÉTAT DU CONTRÔLEUR HAPTIQUE
        # ========================================================================================
        
        # Position cartésienne du contrôleur dans le repère monde [m]
        self.controleur_x = 0.0
        self.controleur_y = 0.0
        self.controleur_z = 0.0
        
        # Orientation du contrôleur en quaternion (qw, qx, qy, qz)
        # Quaternion d'identité par défaut (pas de rotation)
        self.controleur_qx = 0.0
        self.controleur_qy = 0.0
        self.controleur_qz = 0.0
        self.controleur_qw = 1.0

        # ========================================================================================
        # FORCES DE RETOUR HAPTIQUE
        # ========================================================================================
        
        # Forces de translation à appliquer au contrôleur [N]
        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        
        # Couples de rotation à appliquer au contrôleur [N⋅m]
        # Note: Actuellement non utilisés dans cette implémentation
        self.force_rx = 0.0
        self.force_ry = 0.0
        self.force_rz = 0.0
        
        # ========================================================================================
        # DONNÉES DE DISTANCE ROBOT-OBJET
        # ========================================================================================
        
        # Distance euclidienne totale entre robot et objet cible [m]
        self.distance_totale = 0.0
        
        # Distances signées par axe: distance = position_robot - position_objet [m]
        self.distance_x = 0.0  # Positive si robot à droite de l'objet
        self.distance_y = 0.0  # Positive si robot devant l'objet
        self.distance_z = 0.0  # Positive si robot au-dessus de l'objet
        
        # Distances précédentes pour le calcul de dérivées si nécessaire
        self.distance_x_prev = 0.0
        self.distance_y_prev = 0.0
        self.distance_z_prev = 0.0

        # ========================================================================================
        # CALCUL DE VITESSE ET AMORTISSEMENT
        # ========================================================================================
        
        # Position précédente pour le calcul de vitesse [x, y, z] [m]
        self.previous_position = [0.0, 0.0, 0.0]
        
        # Vitesse instantanée calculée [vx, vy, vz] [m/s]
        self.current_velocity = [0.0, 0.0, 0.0]
        
        # Timestamp de la mesure précédente [s]
        self.previous_time = time.time()
        
        # Flag indiquant si les données du contrôleur ont été initialisées
        self.controller_data_initialized = False

        # ========================================================================================
        # PARAMÈTRES DE CONTRÔLE
        # ========================================================================================
        
        # Coefficient d'amortissement pour les translations [N⋅s/m]
        # Plus élevé = plus d'amortissement = mouvements plus stables mais moins réactifs
        self.damping_coeff_translation = 2.0
        
        # ========================================================================================
        # GESTION TEMPORELLE
        # ========================================================================================
        
        # Timestamps pour la gestion des timeouts
        self.last_time = time.time()
        self.last_controller_update_time = time.time()
        
        # Timeout pour considérer les données du contrôleur comme obsolètes [s]
        self.controller_data_timeout = 0.5
        
        # ========================================================================================
        # FLAGS DE CONTRÔLE ET D'ÉTAT
        # ========================================================================================
        
        # Flags indiquant si les données ont été reçues
        self.distance_received = False      # True si distances robot-objet reçues
        self.controller_pose_received = False  # True si pose du contrôleur reçue
        
        # Mode de fonctionnement
        self.in_pose_mode = True    # True: mode position, False: mode vitesse
        self.guide_active = False   # True: guide virtuel activé, False: désactivé
        
        # ========================================================================================
        # VARIABLES DE DEBUG ET MONITORING
        # ========================================================================================
        
        # Compteur pour l'affichage périodique des messages de debug
        self.count = 0
        
        # ========================================================================================
        # TIMER PRINCIPAL
        # ========================================================================================
        
        # Timer pour l'envoi périodique des forces (10Hz = 100ms)
        # Fréquence élevée nécessaire pour un retour haptique fluide
        self.timer = self.create_timer(0.1, self.send_force)
        
        # Message de démarrage
        self.get_logger().info('Nœud de création de guide virtuel démarré')
        self.get_logger().info('Configuration:')
        self.get_logger().info(f'  - Fréquence de calcul: 10Hz')
        self.get_logger().info(f'  - Coefficient d\'amortissement: {self.damping_coeff_translation} N⋅s/m')
        self.get_logger().info(f'  - Timeout données contrôleur: {self.controller_data_timeout}s')
        
    def position_controleur_sub_callback(self, msg):
        """
        Callback pour recevoir et traiter la pose du contrôleur haptique.
        
        Ce callback est appelé à chaque réception d'un message sur le topic /valeur_effecteur.
        Il met à jour la position et l'orientation du contrôleur, calcule la vitesse instantanée
        et met à jour les flags d'état.
        
        Args:
            msg (Float32MultiArray): Message contenant la pose du contrôleur
                Format: [x, y, z, qx, qy, qz, qw] où:
                - x, y, z: position cartésienne [m]
                - qx, qy, qz, qw: quaternion d'orientation
        
        Raises:
            Logs d'erreur si le message ne contient pas assez de données
        """
        # Vérification de l'intégrité du message
        if len(msg.data) < 7:
            self.get_logger().error(
                f"Message valeur_effecteur invalide: attendu au moins 7 valeurs, reçu {len(msg.data)}"
            )
            return
        
        # Extraction de la position cartésienne [m]
        self.controleur_x = float(msg.data[0])
        self.controleur_y = float(msg.data[1])
        self.controleur_z = float(msg.data[2])
        
        # Extraction de l'orientation quaternion
        # Convention: qx, qy, qz, qw (partie vectorielle puis scalaire)
        self.controleur_qx = float(msg.data[3])
        self.controleur_qy = float(msg.data[4])
        self.controleur_qz = float(msg.data[5])
        self.controleur_qw = float(msg.data[6])

        # Calcul de la vitesse instantanée basée sur la dérivée de position
        self.calculate_velocity()
        
        # Mise à jour des flags d'état
        self.controller_pose_received = True
        self.controller_data_initialized = True
        self.last_controller_update_time = time.time()

        # Log de debug pour vérification (affiché occasionnellement)
        if self.count % 100 == 0:
            self.get_logger().debug(
                f"Position contrôleur: [{self.controleur_x:.3f}, {self.controleur_y:.3f}, {self.controleur_z:.3f}]m"
            )
            self.get_logger().debug(
                f"Orientation contrôleur: [{self.controleur_qx:.3f}, {self.controleur_qy:.3f}, "
                f"{self.controleur_qz:.3f}, {self.controleur_qw:.3f}]"
            )

    def mode_callback(self, msg):
        """
        Callback pour recevoir et traiter le mode de contrôle actuel.
        
        Ce callback détermine si le système est en mode position ou vitesse,
        et si le guide virtuel doit être activé ou non.
        
        Args:
            msg (Float32MultiArray): Message de mode
                Format: [pose_flag, vel_flag, x, y, guide_flag] où:
                - pose_flag: 1.0 si mode position, 0.0 sinon
                - vel_flag: 1.0 si mode vitesse, 0.0 sinon
                - x, y: paramètres additionnels (non utilisés ici)
                - guide_flag: 1.0 si guide actif, 0.0 sinon
        
        Note:
            Le guide virtuel n'est actif qu'en mode position avec guide_flag = 1.0
        """
        # Vérification de l'intégrité du message
        if len(msg.data) < 5:
            self.get_logger().error(
                f"Message Mode_Pose_Vitesse invalide: attendu au moins 5 valeurs, reçu {len(msg.data)}"
            )
            return
            
        # Extraction des flags de mode
        # Mode Position: msg.data[0] = 1.0, Mode Vitesse: msg.data[1] = 1.0
        self.in_pose_mode = bool(msg.data[0])
        
        # Guide actif: msg.data[4] = 1.0
        self.guide_active = bool(msg.data[4])
        
        # Log d'information sur le changement de mode
        mode_str = "Position" if self.in_pose_mode else "Vitesse"
        guide_str = "avec guide" if self.guide_active else "sans guide"
        
        self.get_logger().info(f"Mode mis à jour: {mode_str} {guide_str}")
        
        # Log de debug détaillé
        self.get_logger().debug(
            f"Flags reçus: pose={msg.data[0]}, vel={msg.data[1]}, guide={msg.data[4]}"
        )

    def distance_callback(self, msg):
        """
        Callback pour recevoir les distances entre le robot et l'objet cible.
        
        Ces distances sont utilisées pour calculer les forces de guidage qui attirent
        le contrôleur vers l'objet cible.
        
        Args:
            msg (Float32MultiArray): Message de distances
                Format: [distance_totale, dx, dy, dz] où:
                - distance_totale: distance euclidienne totale [m]
                - dx, dy, dz: distances signées par axe [m]
                  (positive si robot plus loin que l'objet dans la direction positive)
        
        Note:
            Les distances signées permettent de déterminer la direction de la force:
            - dx > 0: robot à droite de l'objet → force vers la gauche
            - dy > 0: robot devant l'objet → force vers l'arrière
            - dz > 0: robot au-dessus de l'objet → force vers le bas
        """
        # Vérification de l'intégrité du message
        if len(msg.data) < 4:
            self.get_logger().error(
                f"Message distance_robot_objet invalide: attendu 4 valeurs, reçu {len(msg.data)}"
            )
            return
        
        # Sauvegarde des distances précédentes (pour calculs futurs si nécessaire)
        self.distance_x_prev = self.distance_x
        self.distance_y_prev = self.distance_y
        self.distance_z_prev = self.distance_z
        
        # Mise à jour des nouvelles distances
        self.distance_totale = float(msg.data[0])  # Distance euclidienne [m]
        self.distance_x = float(msg.data[1])       # dx = x_robot - x_objet [m]
        self.distance_y = float(msg.data[2])       # dy = y_robot - y_objet [m]
        self.distance_z = float(msg.data[3])       # dz = z_robot - z_objet [m]
        
        # Flag indiquant que les distances ont été reçues
        self.distance_received = True
        
        # Log de debug périodique pour monitoring
        if self.count % 50 == 0:
            self.get_logger().debug(
                f"Distances reçues: totale={self.distance_totale:.3f}m, "
                f"dx={self.distance_x:.3f}m, dy={self.distance_y:.3f}m, dz={self.distance_z:.3f}m"
            )
        
    def calculate_velocity(self):
        """
        Calcule la vitesse instantanée du contrôleur basée sur la dérivée numérique de position.
        
        Cette méthode utilise la différence finie entre la position actuelle et précédente
        pour estimer la vitesse instantanée. Cette vitesse est ensuite utilisée pour
        calculer les forces d'amortissement.
        
        Méthode:
            v(t) = (x(t) - x(t-dt)) / dt
            
        où dt est l'intervalle de temps entre deux mesures.
        
        Note:
            - Seules les vitesses de translation sont calculées (pas les vitesses angulaires)
            - Un seuil minimum de dt évite la division par zéro
            - La première mesure sert uniquement à initialiser les positions précédentes
        """
        current_time = time.time()
        
        # Calcul de l'intervalle de temps depuis la dernière mesure
        dt = current_time - self.previous_time
        
        # Éviter la division par zéro ou les intervalles trop petits
        if dt < 0.001:  # 1ms minimum
            return
            
        # Initialisation lors de la première mesure
        if not self.controller_data_initialized:
            self.previous_position = [
                self.controleur_x, self.controleur_y, self.controleur_z
            ]
            self.previous_time = current_time
            return
        
        # Position cartésienne actuelle
        current_cartesian = [
            self.controleur_x, self.controleur_y, self.controleur_z
        ]
        
        # Calcul des vitesses par différence finie pour chaque axe
        for i in range(3):  # x, y, z
            self.current_velocity[i] = (current_cartesian[i] - self.previous_position[i]) / dt
        
        # Mise à jour des valeurs précédentes pour la prochaine itération
        self.previous_position = current_cartesian.copy()
        self.previous_time = current_time
        
        # Log de debug périodique des vitesses
        if self.count % 100 == 0:
            self.get_logger().debug(
                f"Vitesses calculées: vx={self.current_velocity[0]:.3f}, "
                f"vy={self.current_velocity[1]:.3f}, vz={self.current_velocity[2]:.3f} [m/s]"
            )
            self.get_logger().debug(f"Intervalle de temps: dt={dt:.4f}s")

    def send_force(self):
        """
        Fonction principale de calcul et d'envoi des forces de retour haptique.
        
        Cette méthode est appelée périodiquement (10Hz) par le timer principal.
        Elle implémente l'algorithme complet de guide virtuel:
        
        1. Vérification des conditions d'activation
        2. Calcul des forces de guidage (proportionnelles à la distance)
        3. Calcul des forces d'amortissement (proportionnelles à la vitesse)
        4. Application des ajustements spécifiques au setup
        5. Limitation des forces et envoi du message
        
        Algorithme de force:
            F_guidage = -k * distance (attraction vers l'objet)
            F_amortissement = -c * vitesse (stabilisation)
            F_totale = F_guidage + F_amortissement
            
        Conditions d'activation:
            - Données de distance et de contrôleur reçues
            - Mode position activé (self.in_pose_mode = True)
            - Guide virtuel activé (self.guide_active = True)
            - Données du contrôleur récentes (< timeout)
        """
        # Vérification des prérequis: données reçues
        if not self.distance_received or not self.controller_pose_received:
            return
            
        # Incrément du compteur pour l'affichage périodique
        self.count += 1
        
        # Initialisation du message de force avec des valeurs nulles
        force_msg = Float32MultiArray()
        force_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # [fx, fy, fz, rx, ry, rz]
        
        # ========================================================================================
        # VÉRIFICATION DES CONDITIONS D'ACTIVATION DU GUIDE
        # ========================================================================================
        
        # Le guide n'est actif qu'en mode position avec guide activé
        if not (self.in_pose_mode and self.guide_active):
            self.guide_force_pub.publish(force_msg)
            
            # Log périodique d'information
            if self.count % 100 == 0:
                self.get_logger().debug(
                    f"Guide inactif - Mode: Pose={self.in_pose_mode}, Guide={self.guide_active}"
                )
            
            return
        
        # Vérification de la fraîcheur des données du contrôleur
        current_time = time.time()
        data_age = current_time - self.last_controller_update_time
        
        if not self.controller_data_initialized or data_age > self.controller_data_timeout:
            # Données trop anciennes: sécurité, arrêt des forces
            self.guide_force_pub.publish(force_msg)
            
            if self.count % 100 == 0:
                self.get_logger().warn(
                    f"Données contrôleur obsolètes (âge: {data_age:.2f}s) - Forces annulées"
                )
            return
        
        # ========================================================================================
        # PARAMÈTRES DE CALCUL DES FORCES
        # ========================================================================================
        
        # Seuil de distance: zone morte où aucune force n'est appliquée [m]
        # Évite les oscillations autour de la cible
        seuil_distance = 0.1  # 10cm
        
        # Facteur de proportionnalité pour les forces de guidage [N/m]
        # Plus élevé = forces plus importantes = guidage plus agressif
        k_force = 3.0
        
        # Force maximale applicable [N]
        # Limitation pour la sécurité et le confort de l'utilisateur
        force_max = 2.0
        
        # ========================================================================================
        # CALCUL DES FORCES PAR AXE
        # ========================================================================================
        
        # Initialisation des forces calculées
        forces = [0.0, 0.0, 0.0]  # [fx, fy, fz]
        distances = [self.distance_x, self.distance_y, self.distance_z]
        
        # Calcul pour chaque axe de translation (x, y, z)
        for axis in range(3):
            # ----- FORCE DE GUIDAGE -----
            # Proportionnelle à la distance, dirigée vers l'objet
            guidage_force = 0.0
            
            if abs(distances[axis]) > seuil_distance:
                # Force négative car distance = robot - objet
                # Si distance > 0: robot plus loin → force négative (vers l'objet)
                # Terme constant (+1.5) pour assurer une force minimale
                guidage_force = -(k_force * distances[axis] + 1.5)
            
            # ----- FORCE D'AMORTISSEMENT -----
            # Proportionnelle à la vitesse, opposée au mouvement
            # Réduit les oscillations et améliore la stabilité
            damping_force = -self.damping_coeff_translation * self.current_velocity[axis]
            
            # ----- FORCE TOTALE -----
            # Combinaison guidage + amortissement
            total_force = guidage_force + damping_force
            
            # Limitation de la force pour sécurité
            total_force = max(-force_max, min(force_max, total_force))
            
            forces[axis] = total_force
        
        # ========================================================================================
        # AJUSTEMENTS SPÉCIFIQUES AU SETUP EXPÉRIMENTAL
        # ========================================================================================
        # Ces ajustements compensent probablement des décalages de repère ou des
        # particularités de la configuration robot/contrôleur
        
        # Ajustement axe X (décalage de 0.25m)
        if (self.controleur_x - 0.25) > 0:
            forces[0] = -forces[0]
            
        # Ajustement axe Y (décalage de -0.02m)
        if (self.controleur_y + 0.02) > 0:
            forces[1] = -forces[1]
            
        # Ajustement axe Z (décalage de 0.05m)
        if (self.controleur_z - 0.05) > 0:
            forces[2] = -forces[2]
        
        # ========================================================================================
        # ASSIGNATION ET ENVOI DES FORCES
        # ========================================================================================
        
        # Assignation des forces de translation calculées
        self.force_x = forces[0]
        self.force_y = forces[1]
        self.force_z = forces[2]
        
        # Pas de couples rotationnels dans cette implémentation
        # Pourrait être étendu pour inclure l'orientation dans le futur
        self.force_rx = 0.0
        self.force_ry = 0.0
        self.force_rz = 0.0
        
        # Création et publication du message de force
        force_msg.data = [
            self.force_x, self.force_y, self.force_z,
            self.force_rx, self.force_ry, self.force_rz
        ]
        self.guide_force_pub.publish(force_msg)
        
        # ========================================================================================
        # LOGGING ET MONITORING
        # ========================================================================================
        
        # Affichage principal (fréquent) - informations essentielles
        if self.count % 10 == 0:
            self.get_logger().info(
                f"Distance totale: {self.distance_totale:.3f}m - "
                f"Forces: [{self.force_x:.3f}, {self.force_y:.3f}, {self.force_z:.3f}]N"
            )
            
        # Affichage détaillé (moins fréquent) - debug complet
        if self.count % 100 == 0:
            self.get_logger().debug(
                f"Vitesses: [{self.current_velocity[0]:.3f}, {self.current_velocity[1]:.3f}, "
                f"{self.current_velocity[2]:.3f}] m/s"
            )
            self.get_logger().debug(
                f"Distances par axe: [{distances[0]:.3f}, {distances[1]:.3f}, {distances[2]:.3f}] m"
            )
            self.get_logger().debug(
                f"Position contrôleur: [{self.controleur_x:.3f}, {self.controleur_y:.3f}, "
                f"{self.controleur_z:.3f}] m"
            )


def main(args=None):
    """
    Fonction principale pour l'exécution du nœud de guide virtuel.
    
    Cette fonction initialise ROS2, crée une instance du nœud GuideVirtuel,
    et maintient le nœud actif jusqu'à interruption (Ctrl+C).
    
    Args:
        args: Arguments de ligne de commande (optionnel)
        
    Workflow:
        1. Initialisation de ROS2
        2. Création du nœud GuideVirtuel
        3. Boucle principale (spin) - traitement des callbacks
        4. Nettoyage et arrêt propre
    """
    # Initialisation du système ROS2
    rclpy.init(args=args)
    
    try:
        # Création de l'instance du nœud
        node = GuideVirtuel()
        
        # Information de démarrage
        node.get_logger().info("Guide virtuel prêt - En attente des données...")
        node.get_logger().info("Appuyez sur Ctrl+C pour arrêter")
        
        # Boucle principale: traitement des messages et callbacks
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # Interruption utilisateur (Ctrl+C)
        print("\nArrêt demandé par l'utilisateur")
        
    finally:
        # Nettoyage: destruction du nœud et arrêt de ROS2
        if 'node' in locals():
            node.get_logger().info("Arrêt du guide virtuel")
            node.destroy_node()
        
        rclpy.shutdown()
        print("Guide virtuel arrêté proprement")


def main(args=None):
    rclpy.init(args=args)
    node = GuideVirtuel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
