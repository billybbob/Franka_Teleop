#!/usr/bin/env python3
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
    def __init__(self):
        super().__init__('franka_ik_solver')
        
        # Charger le modèle URDF du Franka Fr3 à partir du fichier XACRO
        self.get_logger().info("Chargement du modèle Fr3...")
        urdf_string = self.load_urdf_from_xacro()
        
        try:
            # Créer un fichier temporaire pour stocker l'URDF
            with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as tmp_file:
                tmp_file.write(urdf_string.encode())
                urdf_path = tmp_file.name
            
            # Création du modèle et des données pour Pinocchio
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            
            # Nettoyage du fichier temporaire
            os.unlink(urdf_path)
            
            # Obtenir l'ID de l'effecteur final
            try:
                self.ee_id = self.model.getFrameId("fr3_hand_tcp")
                self.get_logger().info(f"Effecteur trouvé: fr3_hand_tcp (ID: {self.ee_id})")
            except:
                # Si le nom exact n'est pas trouvé, chercher un frame similaire
                self.get_logger().warn("Frame 'fr3_hand_tcp' non trouvé. Recherche d'un frame d'effecteur alternatif...")
                for i in range(self.model.nframes):
                    frame_name = self.model.frames[i].name
                    if 'hand' in frame_name.lower() or 'tcp' in frame_name.lower() or 'tool' in frame_name.lower() or 'ee' in frame_name.lower():
                        self.ee_id = i
                        self.get_logger().info(f"Utilisation de '{frame_name}' comme effecteur (ID: {self.ee_id})")
                        break
            
            self.get_logger().info(f"Modèle chargé avec succès: {self.model.name}")
            self.get_logger().info(f"Nombre de degrés de liberté: {self.model.nq}")
            self.get_logger().info(f"Noms des articulations: {[self.model.names[i+1] for i in range(self.model.njoints-1)]}")

            # Création d'un publisher qui envoie les valeurs des 7 articulations du bras (sans la pince)
            self.joint_velocities_pub = self.create_publisher(
                Float32MultiArray,
                '/joint_velocities',
                10
            )

            # Création d'un publisher pour la vitesse de la pince
            self.gripper_velocity_pub = self.create_publisher(
                Float64,
                '/gripper_command_velocities',
                10
            )

            # Créer un subscriber pour l'état actuel des articulations
            self.joint_state_sub = self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_state_callback,
                10)
            
            # Création du subscriber pour les vitesses cartésiennes (modification pour utiliser Float32MultiArray)
            self.cmd_vel_sub = self.create_subscription(
                Float32MultiArray, 
                '/cmd_vel', 
                self.cmd_vel_callback, 
                10
            )

            # Création du subscriber pour la pose de l'effecteur du robot
            self.position_effecteur_robot_sub = self.create_subscription(
                TFMessage, 
                '/position_effecteur_robot', 
                self.position_effecteur_robot_callback, 
                10
            )
            
            # Mémoriser l'état actuel des articulations
            self.current_joint_positions = np.zeros(self.model.nq)
            
            # Initialisation des vitesses cartésiennes (seront mises à jour par cmd_vel)
            self.Vx = 0.0  # Vitesse linéaire en X (m/s)
            self.Vy = 0.0  # Vitesse linéaire en Y (m/s) 
            self.Vz = 0.0  # Vitesse linéaire en Z (m/s)
            self.Rx = 0.0  # Vitesse angulaire autour de X (rad/s)
            self.Ry = 0.0  # Vitesse angulaire autour de Y (rad/s)
            self.Rz = 0.0  # Vitesse angulaire autour de Z (rad/s)
            # Ajout d'une variable pour la vitesse des doigts
            self.Vg = 0.0  # Vitesse de la gâchette pour les doigts (rad/s)

            # Stocker les IDs des articulations des doigts
            self.finger_joint_ids = []
            for i in range(1, self.model.njoints):
                joint_name = self.model.names[i]
                if 'fr3_finger_joint' in joint_name:
                    self.finger_joint_ids.append(i-1)  # -1 car Pinocchio compte à partir de 1
                    self.get_logger().info(f"Articulation de doigt trouvée: {joint_name} (ID: {i-1})")

            # Initialiser la position du robot
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.qx = 0.0
            self.qy = 0.0
            self.qz = 0.0
            self.qw = 1.0  # Quaternion d'identité (pas de rotation)
            
            # Créer un timer pour envoyer périodiquement des commandes de vitesse
            self.timer = self.create_timer(0.1, self.send_cartesian_velocities)  # 10Hz
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors du chargement du modèle: {e}")
    
    def cmd_vel_callback(self, msg):
        """Callback pour recevoir les vitesses cartésiennes depuis le topic /cmd_vel (Float32MultiArray)"""
        try:
            # Vérifier que le message contient assez de données (au moins 6 pour les vitesses cartésiennes)
            if len(msg.data) < 6:
                self.get_logger().error(f"Message cmd_vel invalide: attendu au moins 6 valeurs, reçu {len(msg.data)}")
                return
            
            # Extraire les vitesses linéaires et angulaires du message Float32MultiArray
            self.Vx = float(msg.data[0])
            self.Vy = float(msg.data[1])
            self.Vz = float(msg.data[2])
            
            # Extraire les vitesses angulaires
            self.Rx = float(msg.data[3])
            self.Ry = float(msg.data[4])
            self.Rz = float(msg.data[5])
            
            # Extraire la vitesse de la gâchette si elle est disponible (7ème valeur)
            if len(msg.data) >= 7:
                self.Vg = float(msg.data[6])
                self.get_logger().debug(f"Vitesse de gâchette reçue: {self.Vg}")
            
            self.get_logger().debug(f"Nouvelles vitesses reçues: lin[{self.Vx}, {self.Vy}, {self.Vz}], ang[{self.Rx}, {self.Ry}, {self.Rz}], gâchette[{self.Vg}]")
        except Exception as e:
            self.get_logger().error(f"Erreur lors du traitement du message cmd_vel: {e}")
    
    def load_urdf_from_xacro(self):
        """Charge le modèle URDF à partir du fichier XACRO"""
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
        
        if result.returncode != 0:
            self.get_logger().error(f"Erreur lors de la conversion XACRO: {result.stderr}")
            raise RuntimeError(f"Erreur lors de la conversion XACRO: {result.stderr}")
        
        return result.stdout
    
    def joint_state_callback(self, msg):
        """Callback pour recevoir l'état actuel des articulations"""
        # Mettre à jour l'état actuel des articulations
        for i, name in enumerate(msg.name):
            # Trouver l'index de cette articulation dans le modèle
            try:
                idx = self.model.getJointId(name) - 1  # -1 car Pinocchio compte à partir de 1
                if idx >= 0 and idx < self.model.nq:
                    self.current_joint_positions[idx] = msg.position[i]
            except:
                pass  # Ignorer les articulations qui ne sont pas dans le modèle
    
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
        
        # Calculer et envoyer la nouvelle position si nous avons les deux positions
        if hasattr(self, 'in_pose_mode_') and self.in_pose_mode_ and hasattr(self, 'controller_pose_received_') and self.controller_pose_received_:
            self.calculateAndSendCommand()
        
    def limite_cellule(self):
        """
        Permet de calculer la distance entre la position de l'effecteur du robot
        et les bords de la cellule. Si l'effecteur est à une distance inférieur de 5 cm, alors
        le robot s'arrête et met un message d'erreur dans la console.
        """
        # Vérifier que nous avons bien reçu la position du robot
        if not hasattr(self, 'robot_pose_received_') or not self.robot_pose_received_:
            self.get_logger().warn("Position du robot inconnue, impossible de vérifier les limites de la cellule")
            return True  # Autoriser le mouvement par défaut
        
        # Définir les limites de la cellule (à adapter selon votre cellule réelle)
        # Format: [min_x, max_x, min_y, max_y, min_z, max_z]
        limites_cellule = [-2, 2, -1.5, 1.5, -0.5, 2]  # Exemple de limites en mètres
        
        # Distance de sécurité (5 cm)
        marge_securite = 0.05  # en mètres
        
        # Position actuelle de l'effecteur
        x = self.x  # Utiliser self.x au lieu de self.robot_pose_.x
        y = self.y  # Utiliser self.y au lieu de self.robot_pose_.y
        z = self.z  # Utiliser self.z au lieu de self.robot_pose_.z
        
        # Vérifier si l'effecteur est trop proche des limites
        trop_proche = False
        message = ""
        
        # Vérification sur l'axe X
        if x < limites_cellule[0] + marge_securite:
            trop_proche = True
            message += f"Trop proche de la limite X minimale! (distance: {x - limites_cellule[0]:.3f}m) "
        elif x > limites_cellule[1] - marge_securite:
            trop_proche = True
            message += f"Trop proche de la limite X maximale! (distance: {limites_cellule[1] - x:.3f}m) "
        
        # Vérification sur l'axe Y
        if y < limites_cellule[2] + marge_securite:
            trop_proche = True
            message += f"Trop proche de la limite Y minimale! (distance: {y - limites_cellule[2]:.3f}m) "
        elif y > limites_cellule[3] - marge_securite:
            trop_proche = True
            message += f"Trop proche de la limite Y maximale! (distance: {limites_cellule[3] - y:.3f}m) "
        
        # Vérification sur l'axe Z
        if z < limites_cellule[4] + marge_securite:
            trop_proche = True
            message += f"Trop proche de la limite Z minimale! (distance: {z - limites_cellule[4]:.3f}m) "
        elif z > limites_cellule[5] - marge_securite:
            trop_proche = True
            message += f"Trop proche de la limite Z maximale! (distance: {limites_cellule[5] - z:.3f}m) "
        
        # Si trop proche, logger un message d'erreur et arrêter le robot
        if trop_proche:
            self.get_logger().error(f"ALERTE: {message}")
            # Arrêter le robot en mettant toutes les vitesses à zéro
            self.set_cartesian_velocities(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return False  # Ne pas autoriser le mouvement
        
        return True  # Autoriser le mouvement

    def send_cartesian_velocities(self):
        """
        Calcule et envoie les vitesses articulaires correspondant aux vitesses
        cartésiennes définies dans self.Vx, self.Vy, etc.
        """
        # Vérifier qu'on a reçu au moins une fois l'état des articulations
        if np.all(self.current_joint_positions == 0):
            self.get_logger().warn("Aucune position articulaire reçue, impossible de calculer les vitesses")
            return
        
        # Vérifier les limites de la cellule avant de continuer
        if not self.limite_cellule():
            self.get_logger().warn("Mouvement arrêté : robot trop proche des limites de la cellule")
            return
        
        # Mettre à jour la cinématique directe pour la configuration actuelle
        q_current = self.current_joint_positions
        pin.forwardKinematics(self.model, self.data, q_current)
        pin.updateFramePlacement(self.model, self.data, self.ee_id)
        
        # Créer le vecteur de vitesse cartésienne [vx, vy, vz, wx, wy, wz]
        v_cart = np.array([self.Vx, self.Vy, self.Vz, self.Rx, self.Ry, self.Rz])
        
        # Calculer la Jacobienne
        J = pin.computeFrameJacobian(self.model, self.data, q_current, self.ee_id)
        
        # Facteur d'amortissement pour éviter les singularités
        damp = 1e-6
        
        # Calculer les vitesses articulaires à partir des vitesses cartésiennes
        J_pinv = np.linalg.pinv(J, rcond=damp)
        dq = J_pinv @ v_cart
        
        # Limiter les vitesses articulaires
        max_velocity = 0.5  # rad/s, ajustez selon les spécifications du robot
        dq = np.clip(dq, -max_velocity, max_velocity)
        
        # Créer une liste pour les vitesses articulaires (seulement le bras, pas la pince)
        arm_velocities = list(dq[:7])  # Premières 7 articulations
        
        # Publier les vitesses du bras comme Float32MultiArray
        velocities_msg = Float32MultiArray()
        velocities_msg.data = arm_velocities
        self.joint_velocities_pub.publish(velocities_msg)
        
        # Publier la vitesse de la pince séparément sur /gripper_command_velocity
        gripper_msg = Float64()
        gripper_msg.data = self.Vg
        self.gripper_velocity_pub.publish(gripper_msg)
        
        # Log des vitesses calculées (pour debug)
        self.get_logger().debug(f"Vitesses cartésiennes: [{self.Vx}, {self.Vy}, {self.Vz}, {self.Rx}, {self.Ry}, {self.Rz}]")
        self.get_logger().debug(f"Vitesse gâchette: {self.Vg}")
        self.get_logger().debug(f"Vitesses articulaires calculées (bras): {arm_velocities}")
    
    def target_pose_callback(self, msg):
        """Callback pour recevoir une pose cible et calculer les vitesses articulaires"""
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
        error = pin.log(current_SE3.inverse() * target_SE3).vector
        
        # Calculer la Jacobienne
        J = pin.computeFrameJacobian(self.model, self.data, q_current, self.ee_id)
        
        # Facteur d'amortissement pour éviter les singularités
        damp = 1e-6
        
        # Calculer les vitesses articulaires nécessaires pour se diriger vers la cible
        J_pinv = np.linalg.pinv(J, rcond=damp)
        dq = J_pinv @ error
        
        # Facteur de gain pour ajuster la vitesse
        gain = 0.2  # Ajustez selon vos besoins
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
        """Définit les vitesses cartésiennes à utiliser"""
        self.Vx = vx
        self.Vy = vy
        self.Vz = vz
        self.Rx = rx
        self.Ry = ry
        self.Rz = rz
        
        # Mettre à jour la vitesse de la gâchette si spécifiée
        if vg is not None:
            self.Vg = vg
            self.get_logger().debug(f"Nouvelles vitesses cartésiennes définies: [{vx}, {vy}, {vz}, {rx}, {ry}, {rz}], gâchette: {vg}")
        else:
            self.get_logger().debug(f"Nouvelles vitesses cartésiennes définies: [{vx}, {vy}, {vz}, {rx}, {ry}, {rz}]")
    
    def forward_kinematics(self, q):
        """Calcule la cinématique directe pour une configuration articulaire donnée"""
        # q est un vecteur numpy contenant les angles des articulations
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.ee_id)
        
        # Obtenir la position et l'orientation de l'effecteur final
        ee_position = self.data.oMf[self.ee_id].translation
        ee_orientation = self.data.oMf[self.ee_id].rotation
        
        return ee_position, ee_orientation
    
    def inverse_kinematics(self, target_position, target_orientation, q_init=None):
        """Calcule la cinématique inverse pour atteindre une pose cible"""
        # Si aucune config initiale n'est fournie, utiliser la position actuelle
        if q_init is None:
            q_init = self.current_joint_positions
        
        # Création de la matrice de transformation homogène cible
        target_SE3 = pin.SE3(target_orientation, target_position)
        
        # Tolérance et nombre max d'itérations pour l'IK
        eps = 1e-4
        max_iter = 100
        damp = 1e-6  # Facteur d'amortissement pour éviter les singularités
        
        # Résolution IK par la méthode de Levenberg-Marquardt
        q_result = q_init.copy()
        
        for i in range(max_iter):
            # Calculer la FK pour la configuration actuelle
            pin.forwardKinematics(self.model, self.data, q_result)
            pin.updateFramePlacement(self.model, self.data, self.ee_id)
            current_SE3 = self.data.oMf[self.ee_id]
            
            # Calculer l'erreur (différence entre pose actuelle et cible)
            error = pin.log(current_SE3.inverse() * target_SE3).vector
            
            if np.linalg.norm(error) < eps:
                self.get_logger().info(f"IK convergée après {i} itérations")
                break
                
            # Calculer la Jacobienne
            J = pin.computeFrameJacobian(self.model, self.data, q_result, self.ee_id)
            
            # Mise à jour de la configuration (méthode de Levenberg-Marquardt)
            J_pinv = np.linalg.pinv(J, rcond=damp)
            dq = J_pinv @ error
            q_result += dq
            
            # Normaliser les angles et appliquer les limites articulaires
            q_result = pin.normalize(self.model, q_result)
            
            # Si nous atteignons la dernière itération sans converger
            if i == max_iter - 1:
                self.get_logger().warn("IK n'a pas convergé après le nombre max d'itérations")
        
        return q_result

def main(args=None):
    rclpy.init(args=args)
    
    franka_solver = FrankaIKSolver()
    
    try:
        rclpy.spin(franka_solver)
    except KeyboardInterrupt:
        pass
    finally:
        franka_solver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()