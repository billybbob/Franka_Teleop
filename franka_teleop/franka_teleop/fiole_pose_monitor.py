#!/usr/bin/env python3
"""
Nœud ROS2 pour le monitoring de la pose d'un modèle dans la simulation Ignition Gazebo.

Ce module fait partie du système de téléopération haptique pour le robot Franka FR3.
Il interroge périodiquement la simulation Gazebo à l’aide de la commande `ign topic`
pour extraire la pose d’un modèle (ex: fiole) et republie cette information sur un topic ROS 2
sous forme de message `PoseStamped`.

Auteur: Vincent Bassemayousse
Date: 07/10/2025
Version: 1.0
Licence: Apache 2.0

Dépendances :
    - rclpy : API ROS 2 en Python
    - geometry_msgs.msg.PoseStamped : Message de type pose 3D
    - subprocess, re, threading, time : Modules Python standards
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess
import re
import threading
import time


class FiolePoseMonitor(Node):
    """
    Nœud ROS2 pour la surveillance et la republication de la pose d’un modèle Ignition.

    Fonctionnalités :
    - Utilise la commande shell `ign topic` pour interroger Ignition Gazebo
    - Parse la sortie texte de `ign topic` pour extraire les composantes de pose
    - Publie la pose du modèle comme message ROS 2 `PoseStamped` à fréquence définie

    Paramètres :
        - model_name (str) : Nom du modèle Ignition à suivre (par défaut "Fiole")
        - poll_rate (float) : Fréquence d’interrogation de la pose (Hz)

    Topics publiés :
        - /fiole/pose (geometry_msgs/PoseStamped) : Pose actuelle du modèle
    """

    def __init__(self):
        """
        Initialise le nœud ROS2, les paramètres, le publisher et le thread de polling.
        """
        super().__init__('fiole_pose_monitor')

        # === Paramètres du nœud ===
        self.declare_parameter('model_name', 'Fiole')
        self.declare_parameter('poll_rate', 10.0)

        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.poll_rate = self.get_parameter('poll_rate').get_parameter_value().double_value

        # === Construction du nom du topic Ignition ===
        self.ignition_topic = f"/model/{self.model_name}/pose"

        # === Publisher ROS2 ===
        self.publisher = self.create_publisher(
            PoseStamped,
            '/fiole/pose',
            10  # Taille de la file d’attente
        )

        # Valeurs par défaut en cas d'échec de la commande ign topic
        self.default_pose = {
            'position': {'x': 0.45, 'y': 0.0, 'z': 0.42},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        }

        # === Thread de polling (indépendant du cycle ROS) ===
        self.running = True
        self.polling_thread = threading.Thread(target=self.poll_pose)
        self.polling_thread.daemon = True  # Arrêt automatique avec le processus principal
        self.polling_thread.start()

        self.get_logger().info(f"Monitoring de pose démarré pour le modèle : {self.model_name}")
        self.get_logger().info(f"Topic Ignition observé : {self.ignition_topic}")

    def create_default_pose_msg(self):
        """
        Crée un message PoseStamped avec les valeurs par défaut.
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        
        # Assigner les valeurs par défaut
        pose_msg.pose.position.x = self.default_pose['position']['x']
        pose_msg.pose.position.y = self.default_pose['position']['y']
        pose_msg.pose.position.z = self.default_pose['position']['z']
        
        pose_msg.pose.orientation.x = self.default_pose['orientation']['x']
        pose_msg.pose.orientation.y = self.default_pose['orientation']['y']
        pose_msg.pose.orientation.z = self.default_pose['orientation']['z']
        pose_msg.pose.orientation.w = self.default_pose['orientation']['w']

        return pose_msg

    def poll_pose(self):
        """
        Fonction exécutée dans un thread séparé pour interroger la pose du modèle.

        Utilise la commande `ign topic -e -n 1 -t <topic>` pour récupérer une instance
        du message pose publié par Ignition. Parse la sortie texte, construit un message
        `PoseStamped` ROS 2, et le publie.
        """
        while self.running and rclpy.ok():
            pose_msg = None
            use_default = False

            try:
                # Commande shell Ignition : lecture d’un seul message
                cmd = ["ign", "topic", "-e", "-n", "1", "-t", self.ignition_topic]
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)

                if result.returncode == 0:
                    output = result.stdout

                    # Vérifier que le message contient la pose
                    if "pose {" in output:
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_msg.header.frame_id = "world"

                        # === Extraction des coordonnées position ===
                        x_match = re.search(r"position\s*\{\s*x:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", output)
                        y_match = re.search(r"y:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", output)
                        z_match = re.search(r"z:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", output)

                        # === Extraction des composantes d’orientation ===
                        orientation_section = re.search(r"orientation\s*\{([^}]+)\}", output)
                        if orientation_section:
                            orientation_text = orientation_section.group(1)
                            qx_match = re.search(r"x:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", orientation_text)
                            qy_match = re.search(r"y:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", orientation_text)
                            qz_match = re.search(r"z:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", orientation_text)
                            qw_match = re.search(r"w:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", orientation_text)
                        else:
                            qx_match = qy_match = qz_match = qw_match = None

                        # === Affectation des données extraites ===
                        if x_match and y_match and z_match:
                            pose_msg.pose.position.x = float(x_match.group(1))
                            pose_msg.pose.position.y = float(y_match.group(1))
                            pose_msg.pose.position.z = float(z_match.group(1))

                            pose_msg.pose.orientation.x = float(qx_match.group(1)) if qx_match else 0.0
                            pose_msg.pose.orientation.y = float(qy_match.group(1)) if qy_match else 0.0
                            pose_msg.pose.orientation.z = float(qz_match.group(1)) if qz_match else 0.0
                            pose_msg.pose.orientation.w = float(qw_match.group(1)) if qw_match else 1.0

                            self.publisher.publish(pose_msg)
                            self.get_logger().debug(
                                f"Pose publiée : x={pose_msg.pose.position.x:.3f}, "
                                f"y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}"
                            )
                        else:
                            self.get_logger().warn("Impossible d'extraire les coordonnées de position")
                            use_default = True
                    else:
                        self.get_logger().warn("Format de message 'pose' non reconnu dans la sortie")
                        use_default = True
                else:
                    self.get_logger().error(f"Erreur de commande 'ign topic' : {result.stderr}")
                    use_default = True
            
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Commande 'ign topic' expirée (timeout)")
                use_default = True
            except Exception as e:
                self.get_logger().error(f"Exception dans le polling : {str(e)}")
                use_default = True

            # Si on n'a pas réussi à récupérer la pose ou si elle est invalide, utiliser les valeurs par défaut
            if use_default or pose_msg is None:
                pose_msg = self.create_default_pose_msg()
                self.get_logger().debug(f"Pose par défaut utilisée pour {self.model_name}: x={pose_msg.pose.position.x:.3f}, y={pose_msg.pose.position.y:.3f}, z={pose_msg.pose.position.z:.3f}")
            
            # Publier le message (qu'il vienne d'Ignition ou des valeurs par défaut)
            self.publisher.publish(pose_msg)

            # Délai entre chaque interrogation
            time.sleep(1.0 / self.poll_rate)

    def destroy_node(self):
        """
        Arrête proprement le thread de polling avant de détruire le nœud.
        """
        self.running = False
        if self.polling_thread.is_alive():
            self.polling_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    """
    Fonction principale pour l'exécution du nœud ROS2.
    """
    rclpy.init(args=args)
    fiole_pose_monitor = FiolePoseMonitor()

    try:
        rclpy.spin(fiole_pose_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        fiole_pose_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

