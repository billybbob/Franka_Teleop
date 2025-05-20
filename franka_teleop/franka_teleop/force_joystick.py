#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

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
        
        # Création du subscriber pour le mode (position/vitesse)
        self.mode_sub = self.create_subscription(
            Float32MultiArray,
            '/Mode_Pose_Vitesse',
            self.mode_callback,
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
        self.gachette = 0.0       # Valeur de la gâchette

        # Initialiser la force du contrôleur
        self.force_x = 0.0
        self.force_y = 0.0
        self.force_z = 0.0
        self.force_rx = 0.0
        self.force_ry = 0.0
        self.force_rz = 0.0
        
        # Initialiser les flags de contrôle
        self.controller_pose_received = False
        self.in_speed_mode = False  # Par défaut, on est en mode position
        self.scale_factor = 1.0     # Facteur d'échelle
        
        # Définir les limites pour le retour de force, similaire au code C++
        # Structure: [seuil_inférieur, seuil_supérieur, limite_max, limite_min]
        self.value_for_deplacement = [
            [0.24, 0.26, 0.37, 0.13],           # Axe X translation
            [-0.05, 0.05, 0.23, -0.27],         # Axe Y translation
            [0.05875, 0.11125, 0.19, -0.02],    # Axe Z translation
            [-0.075, 0.375, 0.55, -0.25],       # Rotation qx
            [-0.225, 0.225, 0.9, -0.9],         # Rotation qy
            [-0.225, 0.225, 0.9, -0.9],         # Rotation qz
            [-0.225, 0.225, 0.9, -0.9]          # Rotation qw
        ]
        
        # Définir les constantes de force
        self.max_force_by_axis = [2.0, 2.0, 2.0]      # Force maximale en translation (N)
        self.max_torque_by_axis = [0.1, 0.1, 0.1]     # Couple maximum en rotation (Nm)
        self.stiffness_translation = 100.0            # Coefficient de raideur en translation
        self.stiffness_rotation = 100.0               # Coefficient de raideur en rotation
        
        # Compteur pour les messages de debug
        self.count = 0
        
        # Créer un timer pour appeler périodiquement send_force
        self.timer = self.create_timer(0.001, self.send_force)  # Appel à 1000Hz pour correspondre au code C++
        
        self.get_logger().info('Nœud de création de guide virtuel démarré')

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
        self.gachette = float(msg.data[7])
        
        self.controller_pose_received = True
        
        self.get_logger().debug(f"Position contrôleur reçue: [{self.controleur_x}, {self.controleur_y}, {self.controleur_z}] " +
                               f"[{self.controleur_qx}, {self.controleur_qy}, {self.controleur_qz}, {self.controleur_qw}]")

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
        
        # Tableau pour stocker les valeurs cartésiennes
        cartesian_values = [
            self.controleur_x, self.controleur_y, self.controleur_z,
            self.controleur_qx, self.controleur_qy, self.controleur_qz, self.controleur_qw
        ]
        
        # Vérifier chaque axe et calculer la force si nécessaire
        for axis in range(7):  # 7 axes: 3 translations + 4 quaternion
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
                        force = self.stiffness_translation * normalized_distance * self.max_force_by_axis[axis]
                    elif axis < 6:  # Rotation (uniquement qx, qy, qz)
                        force = self.stiffness_rotation * normalized_distance * self.max_torque_by_axis[axis-3]
                    # Ignorer l'axe 6 (qw)
                    
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
                        force = - (self.stiffness_translation * normalized_distance * self.max_force_by_axis[axis])
                    elif axis < 6:  # Rotation (uniquement qx, qy, qz)
                        force = - (self.stiffness_rotation * normalized_distance * self.max_torque_by_axis[axis-3])
                    # Ignorer l'axe 6 (qw)
                    
                    # Déterminer les zones d'avertissement et critiques
                    if normalized_distance > 0.25:
                        in_warning_zone = True
                    if normalized_distance > 0.6:
                        in_critical_zone = True
            
            # Limiter la force au maximum
            if axis < 3:  # Translation
                if abs(force) > self.max_force_by_axis[axis]:
                    force = self.max_force_by_axis[axis] if force > 0 else -self.max_force_by_axis[axis]
            elif axis < 6:  # Rotation (uniquement qx, qy, qz)
                if abs(force) > self.max_torque_by_axis[axis-3]:
                    force = self.max_torque_by_axis[axis-3] if force > 0 else -self.max_torque_by_axis[axis-3]
            # Ignorer l'axe 6 (qw)
            
            # Assigner la force calculée au bon axe (seulement pour les 6 premiers axes)
            if axis < 6:
                repulsion_force[axis] = force
            
            # Mettre à jour les drapeaux globaux
            if in_critical_zone:
                critical_axis = True
            if in_warning_zone:
                warning_axis = True
            
            # Log debug info every 1000 cycles
            if self.count % 1000 == 0:
                axis_name = ["X trans", "Y trans", "Z trans", "X rot", "Y rot", "Z rot", "W rot"][axis]
                debug_str = f"Axe {axis_name}: Pos={axis_position:.3f}"
                
                if axis < 3:
                    debug_str += f"m Force={force:.3f}N"
                elif axis < 6:  # Ajouter vérification pour qu'il n'affiche pas de force pour qw
                    debug_str += f" Force={force:.3f}Nm"
                
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
        if self.count % 1000 == 0:
            self.get_logger().debug(f"Mode Vitesse - Forces appliquées: F[{repulsion_force[0]:.2f}, {repulsion_force[1]:.2f}, {repulsion_force[2]:.2f}] " +
                                  f"T[{repulsion_force[3]:.2f}, {repulsion_force[4]:.2f}, {repulsion_force[5]:.2f}]")
            
            # Status warning/critical
            if critical_axis:
                self.get_logger().warn("ATTENTION: Axes en zone CRITIQUE")
            elif warning_axis:
                self.get_logger().debug("Axes en zone d'ATTENTION")


def main(args=None):
    rclpy.init(args=args)
    node = Joystick()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()