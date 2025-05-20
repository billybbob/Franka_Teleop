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
from std_msgs.msg import Float32MultiArray, Float64
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrankaMGSolver(Node):
    def __init__(self):
        super().__init__('franka_ig_solver')
        
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

            # Création d'un publisher pour les positions articulaires calculées
            self.joint_positions_pub = self.create_publisher(
                Float32MultiArray,
                '/joint_positions',
                10
            )
            
            # Création d'un publisher pour la commande de la pince
            self.gripper_pub = self.create_publisher(
                Float64,
                '/gripper_command_positions',
                10
            )

            # Créer un subscriber pour l'état actuel des articulations
            self.joint_state_sub = self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_state_callback,
                10)
            
            # Créer un subscriber pour la pose cible
            self.cmd_pose_sub = self.create_subscription(
                Float32MultiArray, 
                '/cmd_pose', 
                self.cmd_pose_callback, 
                10
            )
            
            # Setup TF listener pour obtenir la transformation entre base_link et effecteur
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            
            # Mémoriser l'état actuel des articulations
            self.current_joint_positions = np.zeros(self.model.nq)

            # Initialiser avec les valeurs connues
            self.current_joint_positions = np.array([-2.735252171512916e-07, -0.7854108458715359, 2.7657099024706256e-12, -2.356229067447648, -4.253565360196798e-06, 1.5708193814872145, 0.0])
            
            # Paramètres pour le MGI
            self.mgi_max_iter = self.declare_parameter('mgi_max_iter', 100).value
            self.mgi_epsilon = self.declare_parameter('mgi_epsilon', 1e-4).value
            self.mgi_damping = self.declare_parameter('mgi_damping', 1e-6).value
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors du chargement du modèle: {e}")
    
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
    
    def cmd_pose_callback(self, msg):
        """Callback pour recevoir une commande Float32MultiArray et calculer les angles articulaires par MGI"""
        if len(msg.data) < 7:
            self.get_logger().error(f"Message de pose invalide: attendu au moins 7 valeurs (position XYZ + quaternion XYZW), reçu {len(msg.data)}")
            return
        
        # Extraire la position cible (x, y, z)
        target_position = np.array(msg.data[0:3])
        
        # Extraire le quaternion (x, y, z, w)
        quat_x, quat_y, quat_z, quat_w = msg.data[3:7]
        
        # Normaliser le quaternion
        quat_norm = np.sqrt(quat_x**2 + quat_y**2 + quat_z**2 + quat_w**2)
        if quat_norm < 1e-10:
            self.get_logger().error("Quaternion reçu a une norme nulle ou proche de zéro")
            return
            
        quat_normalized = np.array([quat_x, quat_y, quat_z, quat_w]) / quat_norm
        
        # Convertir le quaternion normalisé en matrice de rotation
        orientation_from_quat = pin.Quaternion(quat_normalized[3], quat_normalized[0], 
                                          quat_normalized[1], quat_normalized[2]).toRotationMatrix()
    
        # Créer une matrice de rotation pour une rotation de pi (180°) autour de l'axe X
        rotation_x_pi = pin.utils.rpyToMatrix(np.pi, 0, 0)  # rotation pi autour de X
        
        # Appliquer la rotation supplémentaire pour aligner les repères
        target_orientation = orientation_from_quat @ rotation_x_pi
        
        self.get_logger().debug(f"Calcul du MGI pour pose: pos={target_position}, quat=[{quat_normalized[0]:.4f}, {quat_normalized[1]:.4f}, {quat_normalized[2]:.4f}, {quat_normalized[3]:.4f}] (avec rotation X 180°)")
        
        # Extraire la valeur de la gâchette (8ème élément) si elle est disponible
        gripper_value = None
        if len(msg.data) >= 8:
            gripper_value = float(msg.data[7])
            self.get_logger().debug(f"Valeur de la pince reçue: {gripper_value}")
            
            # Publier la valeur de la pince sur le topic /gripper_command_positions
            gripper_msg = Float64()
            gripper_msg.data = gripper_value
            self.gripper_pub.publish(gripper_msg)
        
        # Résoudre le MGI
        q_solution = self.geometric_inverse(target_position, target_orientation)
        
        if q_solution is not None:
            # Vérifier la solution en calculant le MGD
            solution_position, solution_orientation = self.geometric_direct(q_solution)
            position_error = np.linalg.norm(target_position - solution_position)
            
            # Pour l'erreur d'orientation, utiliser l'angle entre les deux orientations
            orientation_error_vec = pin.log3(solution_orientation.T @ target_orientation)
            orientation_error = np.linalg.norm(orientation_error_vec)
            
            self.get_logger().info(f"Solution trouvée avec erreurs: position={position_error:.6f}m, orientation={orientation_error:.6f}rad")
            
            # Publier la solution pour les contrôleurs
            self.publish_joint_positions(q_solution)
            
        else:
            self.get_logger().error("Impossible de trouver une solution valide pour la pose cible")
    
    def publish_joint_positions(self, q_solution):
        """Publie les positions articulaires calculées avec le type Float32MultiArray"""
        # Créer un message Float32MultiArray pour les positions articulaires
        joint_cmd = Float32MultiArray()
        
        # Ajouter les positions des articulations (sans les noms)
        joint_cmd.data = [float(q_solution[i]) for i in range(min(len(q_solution), 7))]
        
        # Publier le message
        self.joint_positions_pub.publish(joint_cmd)
    
    def geometric_direct(self, q):
        """Calcule le modèle géométrique direct pour une configuration articulaire donnée"""
        # q est un vecteur numpy contenant les angles des articulations
        if q.shape[0] == 7:
            q = np.concatenate([q, [0.035, 0.035]])

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.ee_id)
        
        # Obtenir la position et l'orientation de l'effecteur final
        ee_position = self.data.oMf[self.ee_id].translation
        ee_orientation = self.data.oMf[self.ee_id].rotation
        
        return ee_position, ee_orientation
    
    def geometric_inverse(self, target_position, target_orientation, q_init=None):
        """
        Calcule le modèle géométrique inverse pour atteindre une pose cible.
        """
        # Si aucune configuration initiale n'est fournie, utiliser la position actuelle
        if q_init is None:
            q_init = self.current_joint_positions.copy()
        
        # Création de la matrice de transformation homogène cible
        target_SE3 = pin.SE3(target_orientation, target_position)
        
        # Résoudre le MGI avec la configuration actuelle comme point de départ
        q_result = self._solve_mgi(target_SE3, q_init)
        
        return q_result
    
    def _solve_mgi(self, target_SE3, q_init):
        """Méthode interne pour résoudre le MGI avec une configuration initiale donnée"""
        # Tolérance et nombre max d'itérations pour le MGI
        eps = self.mgi_epsilon
        max_iter = self.mgi_max_iter
        damp = self.mgi_damping
        
        # Résolution MGI par la méthode de Levenberg-Marquardt
        q_result = q_init.copy()
        
        for i in range(max_iter):
            # Calculer le MGD pour la configuration actuelle
            if q_result.shape[0] == 7:
                q_result = np.concatenate([q_result, [0.035, 0.035]])

            pin.forwardKinematics(self.model, self.data, q_result)
            pin.updateFramePlacement(self.model, self.data, self.ee_id)
            current_SE3 = self.data.oMf[self.ee_id]
            
            # Calculer l'erreur (différence entre pose actuelle et cible)
            error = pin.log(current_SE3.inverse() * target_SE3).vector
            
            error_norm = np.linalg.norm(error)
            
            # Vérifier si on a convergé
            if error_norm < eps:
                self.get_logger().debug(f"MGI convergé après {i+1} itérations avec erreur {error_norm:.6f}")
                return q_result
            
            # Calculer la Jacobienne
            J = pin.computeFrameJacobian(self.model, self.data, q_result, self.ee_id)
            
            # Mise à jour de la configuration (méthode de Levenberg-Marquardt)
            J_pinv = np.linalg.pinv(J, rcond=damp)
            dq = J_pinv @ error
            
            # Pas d'adaptation pour éviter les grands changements
            alpha = min(1.0, 0.5 / max(1e-10, np.linalg.norm(dq)))
            q_result += alpha * dq
            
            # Normaliser les angles et appliquer les limites articulaires
            q_result = pin.normalize(self.model, q_result)
            
            # Appliquer des limites articulaires si disponibles
            if hasattr(self.model, 'lowerPositionLimit') and hasattr(self.model, 'upperPositionLimit'):
                q_result = np.minimum(np.maximum(q_result, self.model.lowerPositionLimit), 
                                     self.model.upperPositionLimit)
        
        # Si nous atteignons la dernière itération sans converger
        self.get_logger().warn(f"MGI n'a pas convergé après {max_iter} itérations. Erreur finale: {error_norm:.6f}")
        
        # Vérifier si la solution est "assez bonne" malgré la non-convergence
        if error_norm < 10*eps:  # Si l'erreur est raisonnablement petite
            self.get_logger().debug(f"Solution approximative trouvée avec erreur {error_norm:.6f}")
            return q_result
        
        return None  # Échec de la convergence

def main(args=None):
    rclpy.init(args=args)
    
    franka_solver = FrankaMGSolver()
    
    try:
        rclpy.spin(franka_solver)
    except KeyboardInterrupt:
        pass
    finally:
        franka_solver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()