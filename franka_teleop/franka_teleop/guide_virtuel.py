#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class GuideVirtuel(Node):
    def __init__(self):
        super().__init__('guide_virtuel')

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

        # Création du subscriber pour les distances robot-objet
        self.distance_sub = self.create_subscription(
            Float32MultiArray,
            '/distance_robot_objet',
            self.distance_callback,
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

        # Initialiser la force du contrôleur
        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        self.force_rx = 0.0
        self.force_ry = 0.0
        self.force_rz = 0.0
        
        # Initialiser les distances robot-objet
        self.distance_totale = 0.0
        self.distance_x = 0.0
        self.distance_y = 0.0
        self.distance_z = 0.0
        
        # Initialiser les distances précédentes
        self.distance_x_prev = 0.0
        self.distance_y_prev = 0.0
        self.distance_z_prev = 0.0

        # Variables pour le calcul de vitesse
        self.previous_position = [0.0, 0.0, 0.0]  # [x, y, z] pour les translations uniquement
        self.current_velocity = [0.0, 0.0, 0.0]   # [vx, vy, vz] pour les translations uniquement
        self.previous_time = time.time()
        self.controller_data_initialized = False

        # Coefficients d'amortissement (inspiré de force_vitesse.py)
        self.damping_coeff_translation = 2.0  # N·s/m
        
        # Variables pour le calcul du dt réel
        self.last_time = time.time()
        self.last_controller_update_time = time.time()
        
        # Timeout pour considérer les données du contrôleur comme obsolètes (en secondes)
        self.controller_data_timeout = 0.5
        
        # Initialiser les flags de contrôle
        self.distance_received = False
        self.controller_pose_received = False
        self.in_pose_mode = True  # Par défaut, on est en mode position
        self.guide_active = False # Par défaut, il n'y a pas de guide
        
        # Compteur pour les messages de debug
        self.count = 0
        
        # Créer un timer pour appeler périodiquement send_force
        self.timer = self.create_timer(0.1, self.send_force)  # Appel à 10Hz
        
        self.get_logger().info('Nœud de création de guide virtuel démarré')
        
    # Callback pour recevoir la position du contrôleur
    def position_controleur_sub_callback(self, msg):
        """Callback pour recevoir la position du contrôleur"""

        # Vérifier que le message contient assez de données (au moins 7 pour x,y,z,qx,qy,qz,qw)
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

        # Calculer la vitesse
        self.calculate_velocity()
        
        self.controller_pose_received = True
        self.controller_data_initialized = True
        self.last_controller_update_time = time.time()

        self.get_logger().debug(f"Position contrôleur reçue: [{self.controleur_x}, {self.controleur_y}, {self.controleur_z}] " +
                               f"[{self.controleur_qx}, {self.controleur_qy}, {self.controleur_qz}, {self.controleur_qw}]")

    # Callback pour recevoir le mode (position/vitesse)
    def mode_callback(self, msg):
        """Callback pour recevoir le mode actuel"""
        if len(msg.data) < 5:
            self.get_logger().error(f"Message Mode_Pose_Vitesse invalide: attendu au moins 5 valeurs, reçu {len(msg.data)}")
            return
            
        # Mode Position et guide = [1.0, 0.0, x, y, 1.0], Mode Vitesse et pas de guide = [0.0, 1.0, x, y, 0.0]
        self.in_pose_mode = bool(msg.data[0])  # 1.0 si en mode position
        self.guide_active = bool(msg.data[4])  # 1.0 si il y a les guides
        
        self.get_logger().debug(f"Mode mis à jour: {'Position avec guide' if (self.in_pose_mode and self.guide_active) else 'Autre mode'}")

    # Callback pour recevoir les distances robot-objet
    def distance_callback(self, msg):
        """Callback pour recevoir les distances entre robot et objet"""
        if len(msg.data) < 4:
            self.get_logger().error(f"Message distance_robot_objet invalide: attendu 4 valeurs, reçu {len(msg.data)}")
            return
        
        # Format du message: [distance_totale, dx, dy, dz]
        self.distance_totale = float(msg.data[0])
        self.distance_x = float(msg.data[1])  # dx = x_robot - x_objet
        self.distance_y = float(msg.data[2])  # dy = y_robot - y_objet
        self.distance_z = float(msg.data[3])  # dz = z_robot - z_objet
        
        self.distance_received = True
        
        self.get_logger().debug(f"Distances reçues: totale={self.distance_totale:.3f}, " +
                               f"dx={self.distance_x:.3f}, dy={self.distance_y:.3f}, dz={self.distance_z:.3f}")
        
    def calculate_velocity(self):
        """Calcule la vitesse basée sur les positions actuelles et précédentes"""
        current_time = time.time()
        
        # Calculer dt
        dt = current_time - self.previous_time
        
        if dt < 0.001:  # Éviter la division par zéro
            return
            
        # Si c'est la première fois, initialiser les positions précédentes
        if not self.controller_data_initialized:
            self.previous_position = [
                self.controleur_x, self.controleur_y, self.controleur_z
            ]
            self.previous_time = current_time
            return
        
        # Calculer les vitesses pour les 3 axes de translation
        current_cartesian = [
            self.controleur_x, self.controleur_y, self.controleur_z
        ]
        
        for i in range(3):
            self.current_velocity[i] = (current_cartesian[i] - self.previous_position[i]) / dt
        
        # Mettre à jour les positions précédentes
        self.previous_position = current_cartesian.copy()
        self.previous_time = current_time
        
        # Debug périodique
        if self.count % 100 == 0:
            self.get_logger().debug(f"Vitesses - Trans: [{self.current_velocity[0]:.3f}, {self.current_velocity[1]:.3f}, {self.current_velocity[2]:.3f}]")

    def send_force(self):
        """
        Calcule la distance entre l'effecteur du robot et l'objet, puis applique une force
        proportionnelle à la distance pour guider vers l'objet + amortissement.
        Les forces sont envoyées uniquement en mode position avec guide actif.
        """
        if not self.distance_received or not self.controller_pose_received:
            return
            
        # Incrémenter le compteur pour affichage périodique
        self.count += 1
        
        # Initialiser les forces à zéro
        force_msg = Float32MultiArray()
        force_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Si on n'est pas en mode position avec guide actif, envoyer des forces nulles
        if not (self.in_pose_mode and self.guide_active):
            self.guide_force_pub.publish(force_msg)
            
            if self.count % 100 == 0:
                self.get_logger().debug(f"Mode: Pose={self.in_pose_mode}, Guide={self.guide_active} - Aucune force appliquée")
            
            return
        
        # Vérifier si les données du contrôleur sont trop anciennes
        current_time = time.time()
        if not self.controller_data_initialized or (current_time - self.last_controller_update_time) > self.controller_data_timeout:
            # Données obsolètes, ne pas calculer de forces
            self.guide_force_pub.publish(force_msg)
            
            if self.count % 100 == 0:
                self.get_logger().debug("Données du contrôleur obsolètes - Forces de guide mises à zéro")
            return
        
        # Initialiser les forces
        forces = [0.0, 0.0, 0.0]  # [fx, fy, fz]
        distances = [self.distance_x, self.distance_y, self.distance_z]
        
        # Seuil de distance en dessous duquel on n'applique pas de force (zone morte)
        seuil_distance = 0.1  # 10cm
        
        # Facteur de proportionnalité pour les forces de guidage
        k_force = 3.0  # Ajustez selon vos besoins
        
        # Force maximale
        force_max = 2.0
        
        # Calculer les forces pour chaque axe
        for axis in range(3):  # 3 axes de translation
            # FORCE DE GUIDAGE (proportionnelle à la distance)
            guidage_force = 0.0
            if abs(distances[axis]) > seuil_distance:
                # Force négative pour aller vers l'objet (distance = robot - objet)
                guidage_force = - ( k_force * distances[axis] + 1.5)
            
            # FORCE D'AMORTISSEMENT (opposée à la vitesse)
            damping_force = -self.damping_coeff_translation * self.current_velocity[axis]
            
            # FORCE TOTALE: Guidage + Amortissement
            total_force = guidage_force + damping_force
            
            # Limiter la force totale
            total_force = max(-force_max, min(force_max, total_force))
            
            forces[axis] = total_force
        
        # Ajustement selon la position du contrôleur
        if (self.controleur_x - 0.25) > 0:
            forces[0] = -forces[0]
            
        if (self.controleur_y + 0.02) > 0:
            forces[1] = -forces[1]
            
        if (self.controleur_z - 0.05) > 0:
            forces[2] = -forces[2]
        
        # Assigner les forces calculées
        self.force_x = forces[0]
        self.force_y = forces[1]
        self.force_z = forces[2]
        
        # Pas de forces rotationnelles pour l'instant
        self.force_rx = 0.0
        self.force_ry = 0.0
        self.force_rz = 0.0
        
        # Créer et envoyer le message de force
        force_msg.data = [self.force_x, self.force_y, self.force_z, self.force_rx, self.force_ry, self.force_rz]
        self.guide_force_pub.publish(force_msg)
        
        # Affichage périodique des informations
        if self.count % 10 == 0:
            self.get_logger().info(f"Distance totale: {self.distance_totale:.3f}m - " +
                                  f"Forces appliquées: [{self.force_x:.3f}, {self.force_y:.3f}, {self.force_z:.3f}]")
            
        # Debug détaillé moins fréquent
        if self.count % 100 == 0:
            self.get_logger().debug(f"Vitesses: [{self.current_velocity[0]:.3f}, {self.current_velocity[1]:.3f}, {self.current_velocity[2]:.3f}] m/s")
            self.get_logger().debug(f"Distances: [{distances[0]:.3f}, {distances[1]:.3f}, {distances[2]:.3f}] m")

def main(args=None):
    rclpy.init(args=args)
    node = GuideVirtuel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()