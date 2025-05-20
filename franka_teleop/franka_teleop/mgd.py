#!/usr/bin/env python3
"""
Node ROS2 pour calculer le Modèle Géométrique Direct (MGD) du Franka Fr3
en utilisant la bibliothèque Pinocchio.
"""

# IMPORTATION DES DIFFÉRENTS ÉLÉMENTS UTILES
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import os
import subprocess
from ament_index_python.packages import get_package_share_directory

class MGDNode(Node):
    """
    Node pour calculer le Modèle Géométrique Direct du robot Franka Fr3
    en utilisant Pinocchio.
    """
    def __init__(self):
        super().__init__('mgd_node')
        
        # Paramètres du robot - valeurs par défaut
        self.r1 = 0.333
        self.r3 = 0.316
        self.r5 = 0.384
        self.d4 = 0.0825
        self.d5 = -0.0825
        
        # État actuel des articulations
        self.q = np.array([-2.8737350679638207e-05, -0.785398157365465,  3.592721522664258e-05, 
                           -2.356194490192345, 2.4004978253540097e-05, 1.5707963267948963, 0.7853981634216485])

        # Pas de temps pour le calcul
        self.dt = 0.005  # 200 Hz

        # Initialisation du modèle Pinocchio
        self.setup_pinocchio_model()

        # Création du subscriber pour les positions articulaires
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_callback, 
            10
        )
        
        # Création d'un publisher qui envoie la position de l'effecteur
        self.position_effecteur_robot_pub = self.create_publisher(
            TFMessage,
            '/position_effecteur_robot',
            10
        )

        # Créer un timer pour calculer et publier périodiquement la position de l'effecteur
        self.position_timer = self.create_timer(self.dt, self.calculate_and_publish_position)

        self.get_logger().info('Node MGD démarré')
        self.get_logger().info(f'Positions articulaires initiales: {self.q}')

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
        
    def setup_pinocchio_model(self):
        """Configure le modèle Pinocchio pour le robot Franka Fr3 à partir du fichier XACRO"""
        try:
            # Charger le modèle URDF à partir du fichier XACRO
            urdf_string = self.load_urdf_from_xacro()
            
            # Charger le modèle Pinocchio à partir de la chaîne URDF
            self.model = pin.buildModelFromXML(urdf_string)
            self.data = self.model.createData()
            
            # Obtenir l'ID du frame de l'effecteur terminal
            # Nom typique pour l'effecteur du Franka Fr3
            ee_frame_name = 'fr3_hand'
            
            # Vérifier si le frame existe
            if self.model.existFrame(ee_frame_name):
                self.ee_id = self.model.getFrameId(ee_frame_name)
                self.get_logger().info(f'Frame de l\'effecteur trouvé: {ee_frame_name}')
            else:
                # Essayer de trouver un frame qui pourrait être l'effecteur
                potential_ee_frames = ['panda_link8', 'fr3_link8', 'panda_hand', 'fr3_hand_tcp']
                frame_found = False
                
                for frame_name in potential_ee_frames:
                    if self.model.existFrame(frame_name):
                        self.ee_id = self.model.getFrameId(frame_name)
                        self.get_logger().info(f'Frame de l\'effecteur alternatif trouvé: {frame_name}')
                        frame_found = True
                        break
                
                if not frame_found:
                    # Utiliser le dernier frame du modèle comme fallback
                    self.ee_id = self.model.nframes - 1
                    ee_frame_name = self.model.frames[self.ee_id].name
                    self.get_logger().warning(f'Aucun frame d\'effecteur connu trouvé. Utilisation du dernier frame: {ee_frame_name}')
            
            self.get_logger().info(f'Modèle Pinocchio chargé avec succès. Nombre d\'articulations: {self.model.nq}')
            
        except Exception as e:
            self.get_logger().error(f'Erreur lors du chargement du modèle Pinocchio: {str(e)}')
            # Fallback en cas d'erreur: utiliser une méthode alternative ou terminer le programme
            raise

    def joint_callback(self, msg):
        """Callback pour mettre à jour les positions articulaires actuelles"""
        # Dictionnaire nom -> valeur
        joint_dict = dict(zip(msg.name, msg.position))
        
        # Ordre correct attendu pour Franka (hors doigts)
        joint_order = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3',
            'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        for i, joint_name in enumerate(joint_order):
            if joint_name in joint_dict:
                self.q[i] = joint_dict[joint_name]
            else:
                self.get_logger().warning(f'Articulation manquante dans le message: {joint_name}')
        
        self.get_logger().debug(f'Positions articulaires mises à jour: {self.q}')

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
        
        Args:
            q: Vecteur numpy contenant les angles des articulations
            
        Returns:
            position_effecteur: Position (x, y, z) de l'effecteur terminal
            orientation_effecteur: Matrice de rotation de l'effecteur terminal
        """
        # Si q contient seulement 6 articulations, l'adapter au modèle complet du Fr3
        # qui possède 7 articulations + 2 doigts (selon le modèle)
        q_full = np.zeros(self.model.nq)
        for i in range(min(len(q), len(q_full))):
            q_full[i] = q[i]
            
        # Si le modèle inclut les pinces (2 doigts supplémentaires)
        if self.model.nq > 7:
            # Position des doigts (typiquement la dernière valeur)
            q_full[7:] = 0.035  # Valeur typique pour les pinces du Fr3
        
        # Calcul de la cinématique directe
        pin.forwardKinematics(self.model, self.data, q_full)
        
        # Mise à jour de la position du frame de l'effecteur terminal
        pin.updateFramePlacement(self.model, self.data, self.ee_id)
        
        # Obtenir la position et l'orientation de l'effecteur final
        position_effecteur = self.data.oMf[self.ee_id].translation
        orientation_effecteur = self.data.oMf[self.ee_id].rotation
        
        return position_effecteur, orientation_effecteur

    def create_transform_stamped(self, position, orientation, 
                                 child_frame='end_effector', parent_frame='base'):
        """
        Crée un message TransformStamped à partir d'une position et d'une orientation.
        
        Args:
            position: Position [x, y, z]
            orientation: Matrice de rotation 3x3
            child_frame: Nom du frame enfant
            parent_frame: Nom du frame parent
            
        Returns:
            TransformStamped: Message de transformation
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        # Position
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])

        # Convertir la matrice de rotation en quaternion
        from scipy.spatial.transform import Rotation as R
        rot = R.from_matrix(orientation)
        quat = rot.as_quat()  # [x, y, z, w]

        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])

        return t
    
def main(args=None):
    rclpy.init(args=args)
    mgd_node = MGDNode()
    try:
        rclpy.spin(mgd_node)
    except KeyboardInterrupt:
        mgd_node.get_logger().info('Node arrêté par l\'utilisateur')
    except Exception as e:
        mgd_node.get_logger().error(f'Erreur dans le node MGD: {str(e)}')
    finally:
        mgd_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()