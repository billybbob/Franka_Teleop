#!/usr/bin/env python3
"""
Nœud ROS2 pour lancer automatiquement l'affichage des caméras de la cellule robotique

Ce module implémente un nœud ROS2 qui lance automatiquement les bridges et viewers
pour les caméras d'une cellule robotique dans une simulation Gazebo Fortress.
Il est conçu pour un projet de téléopération d'un robot Franka Fr3 avec contrôleur
haptique Desktop 6D de Haption.

Compatible avec les fichiers launch ROS2 et ROS2 Humble.

Fonctionnalités:
- Lancement automatique des bridges ROS-Gazebo pour les caméras
- Affichage des flux vidéo via image_tools
- Monitoring des processus lancés
- Nettoyage automatique des ressources
- Configuration via paramètres ROS2

Auteur: Vincent Bassemayousse
Date: 07/10/2025
Version: 1.0
Licence: Apache 2.0
Dépendances: ROS2 Humble, Gazebo Fortress
"""

import rclpy
from rclpy.node import Node
import subprocess
import sys
import time
import os
from typing import List, Dict, Optional

class CameraLauncherNode(Node):
    """
    Nœud ROS2 pour la gestion automatique des caméras de la cellule robotique.
    
    Ce nœud gère le lancement et la surveillance des bridges ROS-Gazebo ainsi que
    des viewers pour les caméras gauche et droite de la cellule de téléopération.
    
    Attributes:
        world_name (str): Nom du monde Gazebo dans lequel évoluent les caméras
        model_name (str): Nom du modèle contenant les caméras dans Gazebo
        processes (List[subprocess.Popen]): Liste des processus en cours d'exécution
        cameras (Dict): Configuration des caméras avec leurs paramètres
        cameras_launched (bool): Indicateur de lancement des caméras
        launch_delay (float): Délai entre le lancement du bridge et du viewer
        timer (rclpy.timer.Timer): Timer pour le lancement des caméras
        monitor_timer (rclpy.timer.Timer): Timer pour surveiller les processus
    """
    
    def __init__(self):
        """
        Initialise le nœud CameraLauncher.
        
        Configure les paramètres par défaut, déclare les paramètres ROS2,
        initialise les structures de données et démarre les timers.
        """
        super().__init__('camera_launcher_node')
        
        # Configuration par défaut de la simulation
        self.world_name = "demo"  # Nom du monde Gazebo
        self.model_name = "Cellule"  # Nom du modèle dans Gazebo
        self.processes: List[subprocess.Popen] = []  # Liste des processus actifs
        
        # Configuration des caméras de la cellule robotique
        # Chaque caméra a un lien, un capteur et un suffixe de topic
        self.cameras = {
            "gauche": {
                "link": "Camera_gauche",        # Nom du lien dans le modèle Gazebo
                "sensor": "camera_sensor",      # Nom du capteur dans Gazebo
                "topic_suffix": "gauche"        # Suffixe pour le topic ROS2
            },
            "droite": {
                "link": "Camera_droite", 
                "sensor": "camera_sensor",
                "topic_suffix": "droite"
            }
        }
        
        # Déclaration des paramètres ROS2 configurables
        self.declare_parameter('world_name', self.world_name)
        self.declare_parameter('model_name', self.model_name)
        self.declare_parameter('launch_delay', 2.0)  # Délai entre bridge et viewer (sec)
        
        # Récupération des valeurs des paramètres
        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.launch_delay = self.get_parameter('launch_delay').get_parameter_value().double_value
        
        # Affichage des informations de configuration
        self.get_logger().info(f"Démarrage du lanceur de caméras")
        self.get_logger().info(f"   Monde: {self.world_name}")
        self.get_logger().info(f"   Modèle: {self.model_name}")
        
        # Timer pour lancer les caméras après un délai d'initialisation
        # Le délai de 3 secondes permet à Gazebo de s'initialiser complètement
        self.timer = self.create_timer(3.0, self.launch_cameras_callback)
        self.cameras_launched = False  # Flag pour éviter les lancements multiples
        
        # Timer de monitoring des processus (vérification toutes les 5 secondes)
        self.monitor_timer = self.create_timer(5.0, self.monitor_processes_callback)
    
    def create_bridge_command(self, camera_config: Dict) -> List[str]:
        """
        Crée la commande pour le bridge ROS-Gazebo d'une caméra.
        
        Le bridge ros_gz_bridge permet de faire communiquer les topics Gazebo
        (format ignition.msgs) avec les topics ROS2 (format sensor_msgs).
        
        Args:
            camera_config (Dict): Configuration de la caméra contenant:
                - link: nom du lien dans le modèle Gazebo
                - sensor: nom du capteur dans Gazebo
                - topic_suffix: suffixe pour le topic ROS2
        
        Returns:
            List[str]: Commande complète pour lancer le bridge
            
        Example:
            Pour la caméra gauche:
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/world/demo/model/Cellule/link/Camera_gauche/sensor/camera_sensor/image@sensor_msgs/msg/Image[ignition.msgs.Image',
             '--ros-args', '-r', '/world/demo/model/.../image:=/camera_gauche/image']
        """
        # Construction du topic Gazebo selon la nomenclature Ignition/Gazebo
        topic_gz = f"/world/{self.world_name}/model/{self.model_name}/link/{camera_config['link']}/sensor/{camera_config['sensor']}/image"
        
        # Construction du topic ROS2 de destination
        topic_ros = f"/camera_{camera_config['topic_suffix']}/image"
        
        return [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"{topic_gz}@sensor_msgs/msg/Image[ignition.msgs.Image",  # Mapping des types de messages
            "--ros-args",
            "-r", f"{topic_gz}:={topic_ros}"  # Remapping du topic
        ]
    
    def create_viewer_command(self, camera_config: Dict) -> List[str]:
        """
        Crée la commande pour l'affichage de l'image d'une caméra.
        
        Utilise l'outil showimage du package image_tools pour afficher
        le flux vidéo de la caméra dans une fenêtre graphique.
        
        Args:
            camera_config (Dict): Configuration de la caméra
        
        Returns:
            List[str]: Commande complète pour lancer le viewer
            
        Example:
            ['ros2', 'run', 'image_tools', 'showimage',
             '--ros-args', '-r', 'image:=/camera_gauche/image']
        """
        topic_ros = f"/camera_{camera_config['topic_suffix']}/image"
        
        return [
            "ros2", "run", "image_tools", "showimage",
            "--ros-args",
            "-r", f"image:={topic_ros}"  # Remapping vers le topic de la caméra
        ]
    
    def launch_camera_pair(self, camera_name: str, camera_config: Dict) -> bool:
        """
        Lance le bridge et le viewer pour une caméra donnée.
        
        Cette méthode lance séquentiellement:
        1. Le bridge ROS-Gazebo pour convertir les messages
        2. Le viewer pour afficher l'image (après un délai)
        
        Le délai entre les deux lancements permet au bridge de s'initialiser
        complètement avant que le viewer ne tente de s'abonner au topic.
        
        Args:
            camera_name (str): Nom de la caméra ("gauche" ou "droite")
            camera_config (Dict): Configuration de la caméra
            
        Returns:
            bool: True si le lancement a réussi, False sinon
            
        Raises:
            Exception: Si une erreur survient pendant le lancement
        """
        self.get_logger().info(f"Lancement de la caméra {camera_name}...")
        
        try:
            # === Lancement du bridge ROS-Gazebo ===
            bridge_cmd = self.create_bridge_command(camera_config)
            self.get_logger().debug(f"   Bridge: {' '.join(bridge_cmd)}")
            
            # Lancement du processus bridge avec héritage de l'environnement ROS2
            bridge_process = subprocess.Popen(
                bridge_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=os.environ.copy()  # Hériter de l'environnement ROS2 (ROS_DOMAIN_ID, etc.)
            )
            self.processes.append(bridge_process)
            
            # Attendre que le bridge s'initialise complètement
            # Ce délai est crucial pour éviter les erreurs de synchronisation
            time.sleep(self.launch_delay)
            
            # === Lancement du viewer ===
            viewer_cmd = self.create_viewer_command(camera_config)
            self.get_logger().debug(f"   Viewer: {' '.join(viewer_cmd)}")
            
            # Lancement du processus viewer
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
        """
        Callback du timer pour lancer toutes les caméras.
        
        Cette méthode est appelée une seule fois après le délai d'initialisation.
        Elle lance successivement toutes les caméras configurées avec un délai
        entre chaque lancement pour éviter la surcharge système.
        
        Le timer est annulé après le lancement pour éviter les relances.
        """
        # Vérifier si les caméras ont déjà été lancées
        if self.cameras_launched:
            return
            
        self.get_logger().debug("-" * 50)
        success_count = 0
        
        # Lancement séquentiel de toutes les caméras
        for camera_name, camera_config in self.cameras.items():
            if self.launch_camera_pair(camera_name, camera_config):
                success_count += 1
            time.sleep(1)  # Délai entre les caméras pour éviter la surcharge
        
        # Affichage du bilan de lancement
        self.get_logger().debug("-" * 50)
        self.get_logger().debug(f"{success_count}/{len(self.cameras)} caméras lancées avec succès")
        
        # Affichage des topics créés si au moins une caméra a été lancée
        if success_count > 0:
            self.get_logger().debug("Topics créés:")
            for camera_name in self.cameras.keys():
                self.get_logger().debug(f"   /camera_{camera_name}/image")
        
        # Marquer les caméras comme lancées
        self.cameras_launched = True
        
        # Arrêter le timer de lancement (pas besoin de relancer)
        self.timer.cancel()
    
    def monitor_processes_callback(self):
        """
        Callback du timer pour surveiller l'état des processus.
        
        Cette méthode vérifie périodiquement que tous les processus
        (bridges et viewers) sont encore actifs. Elle log les processus
        qui se sont arrêtés de manière inattendue.
        
        Le monitoring ne commence qu'après le lancement des caméras
        et s'arrête si tous les processus sont terminés.
        """
        # Ne pas surveiller si les caméras n'ont pas encore été lancées
        if not self.cameras_launched or not self.processes:
            return
            
        active_processes = 0
        
        # Vérifier l'état de chaque processus
        for i, process in enumerate(self.processes):
            if process.poll() is None:  # Processus encore actif
                active_processes += 1
            else:
                # Processus terminé - vérifier le code de retour
                if process.returncode != 0:
                    self.get_logger().warn(f"Un processus s'est arrêté avec le code: {process.returncode}")
        
        # Alerte si tous les processus sont arrêtés
        if active_processes == 0:
            self.get_logger().warn("Tous les processus caméra se sont arrêtés")
    
    def cleanup_processes(self):
        """
        Nettoie tous les processus lancés de manière propre.
        
        Cette méthode termine gracieusement tous les processus:
        1. Envoi du signal SIGTERM (terminaison propre)
        2. Attente de la terminaison avec timeout
        3. Envoi du signal SIGKILL si nécessaire (terminaison forcée)
        
        Elle est appelée automatiquement lors de la destruction du nœud
        ou lors d'une interruption clavier (Ctrl+C).
        """
        self.get_logger().info("Arrêt des processus caméra...")
        
        for process in self.processes:
            if process.poll() is None:  # Processus encore actif
                try:
                    # Tentative de terminaison propre
                    process.terminate()
                    process.wait(timeout=5)  # Attendre max 5 secondes
                except subprocess.TimeoutExpired:
                    # Terminaison forcée si le processus ne répond pas
                    process.kill()
        
        # Vider la liste des processus
        self.processes.clear()
        self.get_logger().info("Tous les processus ont été arrêtés")
    
    def destroy_node(self):
        """
        Nettoyage lors de la destruction du nœud.
        
        Cette méthode est appelée automatiquement lors de la destruction
        du nœud ROS2. Elle s'assure que tous les processus sont proprement
        terminés avant la destruction complète.
        """
        self.cleanup_processes()
        super().destroy_node()

def main(args=None):
    """
    Fonction principale du nœud ROS2.
    
    Cette fonction:
    1. Initialise le contexte ROS2
    2. Crée et lance le nœud CameraLauncher
    3. Gère la boucle d'événements ROS2
    4. Effectue le nettoyage en cas d'arrêt ou d'erreur
    
    Args:
        args: Arguments de ligne de commande (optionnel)
        
    Returns:
        int: Code de retour (0 pour succès, 1 pour erreur)
        
    Example:
        Lancement standard:
        $ python3 camera_launcher.py
        
        Lancement avec paramètres:
        $ ros2 run mon_package camera_launcher --ros-args -p world_name:=mon_monde
    """
    # Initialisation du contexte ROS2
    rclpy.init(args=args)
    
    try:
        # Création du nœud
        node = CameraLauncherNode()
        
        try:
            # Lancement de la boucle d'événements ROS2
            # Cette boucle traite les callbacks des timers et les messages
            rclpy.spin(node)
        except KeyboardInterrupt:
            # Gestion de l'interruption clavier (Ctrl+C)
            node.get_logger().info("Arrêt demandé par l'utilisateur")
        finally:
            # Nettoyage des ressources (processus et nœud)
            node.cleanup_processes()
            node.destroy_node()
            
    except Exception as e:
        # Gestion des erreurs de lancement
        print(f"Erreur lors du lancement du nœud: {e}")
        return 1
    finally:
        # Arrêt propre du contexte ROS2
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    """
    Point d'entrée du script.
    
    Permet l'exécution directe du script avec:
    $ python3 camera_launcher.py
    """
    sys.exit(main())