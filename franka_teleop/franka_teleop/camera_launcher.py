#!/usr/bin/env python3
"""
Nœud ROS2 pour lancer automatiquement l'affichage des caméras de la cellule
Compatible avec les fichiers launch ROS2
Auteur: Vincent Bassemayousse
"""

import rclpy
from rclpy.node import Node
import subprocess
import sys
import time
import os
from typing import List, Dict

class CameraLauncherNode(Node):
    def __init__(self):
        super().__init__('camera_launcher_node')
        
        # Configuration
        self.world_name = "demo"
        self.model_name = "Cellule"
        self.processes: List[subprocess.Popen] = []
        
        # Configuration des caméras
        self.cameras = {
            "gauche": {
                "link": "Camera_gauche",
                "sensor": "camera_sensor",
                "topic_suffix": "gauche"
            },
            "droite": {
                "link": "Camera_droite", 
                "sensor": "camera_sensor",
                "topic_suffix": "droite"
            }
        }
        
        # Paramètres ROS2
        self.declare_parameter('world_name', self.world_name)
        self.declare_parameter('model_name', self.model_name)
        self.declare_parameter('launch_delay', 2.0)  # Délai entre bridge et viewer
        
        # Récupérer les paramètres
        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.launch_delay = self.get_parameter('launch_delay').get_parameter_value().double_value
        
        self.get_logger().info(f"Démarrage du lanceur de caméras")
        self.get_logger().info(f"   Monde: {self.world_name}")
        self.get_logger().info(f"   Modèle: {self.model_name}")
        
        # Timer pour lancer les caméras après un délai
        self.timer = self.create_timer(3.0, self.launch_cameras_callback)
        self.cameras_launched = False
        
        # Timer de monitoring
        self.monitor_timer = self.create_timer(5.0, self.monitor_processes_callback)
    
    def create_bridge_command(self, camera_config: Dict) -> List[str]:
        """Crée la commande pour le bridge ROS-Gazebo"""
        topic_gz = f"/world/{self.world_name}/model/{self.model_name}/link/{camera_config['link']}/sensor/{camera_config['sensor']}/image"
        topic_ros = f"/camera_{camera_config['topic_suffix']}/image"
        
        return [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"{topic_gz}@sensor_msgs/msg/Image[ignition.msgs.Image",
            "--ros-args",
            "-r", f"{topic_gz}:={topic_ros}"
        ]
    
    def create_viewer_command(self, camera_config: Dict) -> List[str]:
        """Crée la commande pour l'affichage de l'image"""
        topic_ros = f"/camera_{camera_config['topic_suffix']}/image"
        
        return [
            "ros2", "run", "image_tools", "showimage",
            "--ros-args",
            "-r", f"image:={topic_ros}"
        ]
    
    def launch_camera_pair(self, camera_name: str, camera_config: Dict) -> bool:
        """Lance le bridge et le viewer pour une caméra"""
        self.get_logger().info(f"Lancement de la caméra {camera_name}...")
        
        try:
            # Lancement du bridge
            bridge_cmd = self.create_bridge_command(camera_config)
            self.get_logger().debug(f"   Bridge: {' '.join(bridge_cmd)}")
            
            bridge_process = subprocess.Popen(
                bridge_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=os.environ.copy()  # Hériter de l'environnement ROS2
            )
            self.processes.append(bridge_process)
            
            # Attendre que le bridge s'initialise
            time.sleep(self.launch_delay)
            
            # Lancement du viewer
            viewer_cmd = self.create_viewer_command(camera_config)
            self.get_logger().debug(f"   Viewer: {' '.join(viewer_cmd)}")
            
            viewer_process = subprocess.Popen(
                viewer_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=os.environ.copy()  # Hériter de l'environnement ROS2
            )
            self.processes.append(viewer_process)
            
            self.get_logger().info(f"Caméra {camera_name} lancée avec succès")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors du lancement de la caméra {camera_name}: {e}")
            return False
    
    def launch_cameras_callback(self):
        """Callback pour lancer toutes les caméras"""
        if self.cameras_launched:
            return
            
        self.get_logger().debug("-" * 50)
        success_count = 0
        
        for camera_name, camera_config in self.cameras.items():
            if self.launch_camera_pair(camera_name, camera_config):
                success_count += 1
            time.sleep(1)  # Délai entre les caméras
        
        self.get_logger().debug("-" * 50)
        self.get_logger().debug(f"{success_count}/{len(self.cameras)} caméras lancées avec succès")
        
        if success_count > 0:
            self.get_logger().debug("Topics créés:")
            for camera_name in self.cameras.keys():
                self.get_logger().debug(f"   /camera_{camera_name}/image")
        
        self.cameras_launched = True
        
        # Arrêter le timer de lancement
        self.timer.cancel()
    
    def monitor_processes_callback(self):
        """Surveille les processus en cours"""
        if not self.cameras_launched or not self.processes:
            return
            
        active_processes = 0
        for i, process in enumerate(self.processes):
            if process.poll() is None:
                active_processes += 1
            else:
                if process.returncode != 0:
                    self.get_logger().warn(f"Un processus s'est arrêté avec le code: {process.returncode}")
        
        if active_processes == 0:
            self.get_logger().warn("Tous les processus caméra se sont arrêtés")
    
    def cleanup_processes(self):
        """Nettoie tous les processus lancés"""
        self.get_logger().info("Arrêt des processus caméra...")
        
        for process in self.processes:
            if process.poll() is None:
                try:
                    process.terminate()
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
        
        self.processes.clear()
        self.get_logger().info("Tous les processus ont été arrêtés")
    
    def destroy_node(self):
        """Nettoyage lors de la destruction du nœud"""
        self.cleanup_processes()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraLauncherNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Arrêt demandé par l'utilisateur")
        finally:
            node.cleanup_processes()
            node.destroy_node()
            
    except Exception as e:
        print(f"Erreur lors du lancement du nœud: {e}")
        return 1
    finally:
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())