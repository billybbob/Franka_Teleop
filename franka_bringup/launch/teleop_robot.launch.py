#!/usr/bin/env python3
"""
Fichier de lancement principal pour le système de téléopération du robot Franka Fr3.

Ce module configure et lance tous les composants nécessaires pour la téléopération
d'un robot Franka Fr3 avec un contrôleur haptique Desktop 6D de Haption.
Le système fonctionne en simulation sur Gazebo Fortress avec ROS2 Humble.

Fonctionnalités principales :
- Configuration multi-robot basée sur un fichier YAML
- Lancement automatique des contrôleurs (position, vitesse, gripper)
- Intégration du retour de force haptique
- Surveillance des obstacles et guidage virtuel
- Calculs de cinématique directe et inverse
- Commutation entre différents modes de contrôle

Auteur: Vincent Bassemayousse
Date: 07/10/2025
Version: 1.0
Licence: Apache 2.0
"""

#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Configuration du chemin vers les utilitaires du package
package_share = get_package_share_directory('franka_bringup')
utils_path = os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')
sys.path.append(os.path.abspath(utils_path))

# Import des utilitaires pour charger les fichiers YAML
from launch_utils import load_yaml  # noqa: E402


def generate_robot_nodes(context):
    """
    Génère dynamiquement les nœuds ROS2 pour chaque robot configuré.
    
    Cette fonction lit le fichier de configuration YAML et crée tous les nœuds
    nécessaires pour chaque robot Franka défini dans la configuration.
    
    Args:
        context: Contexte de lancement ROS2 contenant les paramètres configurés
        
    Returns:
        list: Liste des nœuds ROS2 à lancer
        
    Configuration attendue dans le fichier YAML :
        robot_name:
            namespace: "/robot_namespace"
            arm_id: "panda"
            arm_prefix: "panda_"
            urdf_file: "path/to/urdf"
            robot_ip: "192.168.1.100"
            load_gripper: true
            use_fake_hardware: true
            fake_sensor_commands: true
            joint_sources: ["joint_state_broadcaster"]
            joint_state_rate: 50
            use_rviz: false
    """
    # Récupération des paramètres de configuration
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    controller_name = LaunchConfiguration('controller_name').perform(context)
    gripper_controller_name = LaunchConfiguration('gripper_example_controller').perform(context)
    cartesian_velocity_controller_name = LaunchConfiguration('cartesian_velocity_example_controller').perform(context)
    
    # Chargement de la configuration depuis le fichier YAML
    configs = load_yaml(config_file)
    nodes = []

    # Génération des nœuds pour chaque robot configuré
    for item_name, config in configs.items():
        namespace = config['namespace']
        
        # ===== LANCEMENT DU ROBOT DE BASE =====
        # Inclusion du fichier de lancement principal du robot Franka
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'
                    ])
                ),
                launch_arguments={
                    'arm_id': str(config['arm_id']),                    # Identifiant du bras robotique
                    'arm_prefix': str(config['arm_prefix']),            # Préfixe pour les topics/services
                    'namespace': str(namespace),                        # Namespace ROS2
                    'urdf_file': str(config['urdf_file']),             # Fichier de description URDF
                    'robot_ip': str(config['robot_ip']),               # Adresse IP du robot réel
                    'load_gripper': str(config['load_gripper']),       # Activation du gripper
                    'use_fake_hardware': str(config['use_fake_hardware']), # Mode simulation
                    'fake_sensor_commands': str(config['fake_sensor_commands']), # Capteurs simulés
                    'joint_sources': ','.join(config['joint_sources']), # Sources des états articulaires
                    'joint_state_rate': str(config['joint_state_rate']), # Fréquence des états articulaires
                }.items(),
            )
        )

        # ===== CONTRÔLEUR PRINCIPAL =====
        # Spawner pour le contrôleur principal (position/vitesse)
        nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=[controller_name, '--controller-manager-timeout', '30'],
                parameters=[PathJoinSubstitution([
                    FindPackageShare('franka_bringup'), 'config', "controllers.yaml",
                ])],
                output='screen',
            )
        )
        
        # ===== CONTRÔLEUR DU GRIPPER =====
        # Spawner pour le contrôleur du gripper (pince)
        nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=[gripper_controller_name, '--controller-manager-timeout', '30'],
                parameters=[PathJoinSubstitution([
                    FindPackageShare('franka_bringup'), 'config', "controllers.yaml",
                ])],
                output='screen',
            )
        )
        
        # ===== CONTRÔLEUR DE VITESSE CARTÉSIENNE =====
        # Spawner pour le contrôleur de vitesse cartésienne (mode téléopération)
        # Lancé en mode inactif, sera activé selon les besoins
        nodes.append(
            Node(
                package='controller_manager',
                executable='spawner',
                namespace=namespace,
                arguments=[cartesian_velocity_controller_name, '--inactive', '--controller-manager-timeout', '30'],
                parameters=[PathJoinSubstitution([
                    FindPackageShare('franka_bringup'), 'config', "controllers.yaml",
                ])],
                output='screen',
            )
        )

    # ===== INTERFACE DE VISUALISATION =====
    # Lancement optionnel de RViz si configuré dans au moins un robot
    if any(str(config.get('use_rviz', 'false')).lower() == 'true' for config in configs.values()):
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['--display-config', PathJoinSubstitution([
                    FindPackageShare('franka_description'), 'rviz', 'visualize_franka.rviz'
                ])],
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    """
    Génère la description de lancement complète pour le système de téléopération.
    
    Cette fonction configure tous les paramètres de lancement et crée tous les nœuds
    nécessaires pour le système de téléopération du robot Franka Fr3 avec retour
    de force haptique.
    
    Returns:
        LaunchDescription: Description complète du lancement ROS2
        
    Architecture du système :
        1. Robot Franka Fr3 avec ses contrôleurs
        2. Interface haptique Desktop 6D Haption
        3. Modules de traitement (cinématique, forces, vitesses)
        4. Système de surveillance et guidage virtuel
        5. Détection d'obstacles et calcul de distances
    """
    
    # ===== CONFIGURATION GÉNÉRALE =====
    # Paramètre pour la synchronisation temporelle avec Gazebo
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ===== MODULES DE TRAITEMENT DES DONNÉES =====
    
    # Nœud pour le calcul des offsets de position
    # Traite les données de position pour la téléopération
    offset_position = Node(
        package='test_cartesien',
        executable='offset_position',
        name='offset_position',
        output='screen'
    )

    # Nœud pour la conversion valeur vers vitesse
    # Convertit les valeurs de position en commandes de vitesse
    value_to_speed = Node(
        package='test_cartesien',
        executable='value_to_speed',
        name='value_to_speed',
        output='screen'
    )

    # ===== INTERFACE HAPTIQUE ET RETOUR DE FORCE =====
    
    # Nœud pour le retour de force basé sur la vitesse
    # Calcule les forces de retour pour le contrôleur haptique
    force_vitesse_node = Node(
        package='franka_teleop',
        executable='force_vitesse',
        name='force_vitesse_node',
        output='screen'
    )

    # Nœud pour le retour de force basé sur la position
    # Alternative au mode vitesse pour le retour haptique
    force_position_node = Node(
        package='franka_teleop',
        executable='force_position',
        name='force_position_node',
        output='screen'
    )

    # ===== CALCULS CINÉMATIQUES =====
    
    # Nœud pour la résolution de la cinématique inverse géométrique (IG)
    # Calcule les angles articulaires à partir de la position cartésienne
    ig_solver_node = Node(
        package='franka_teleop',
        executable='franka_ig_solver',
        name='franka_ig_solver',
        output='screen'
    )

    # Nœud pour le modèle géométrique direct (MGD)
    # Calcule la position cartésienne à partir des angles articulaires
    mgd_node = Node(
        package='franka_teleop',
        executable='mgd',
        name='mgd_node',
        output='screen'
    )

    # ===== GESTION DES MODES DE CONTRÔLE =====
    
    # Nœud pour la commutation entre différents modes de contrôle
    # Permet de passer entre contrôle en position, vitesse, force, etc.
    controller_switcher_node = Node(
        package='franka_teleop',
        executable='switch_mode',
        name='controller_switcher',
        output='screen'
    )

    # ===== SURVEILLANCE DE L'ENVIRONNEMENT =====
    
    # Nœud pour surveiller la position de la fiole dans la simulation
    # Suit la position de l'objet cible en temps réel
    fiole_pose_monitor = Node(
        package="franka_teleop",
        executable="fiole_pose_monitor",
        name="fiole_pose_monitor",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,    # Synchronisation avec Gazebo
            "object_name": "Fiole_world",    # Nom de l'objet à surveiller
            "poll_rate": 10.0                # Fréquence de mise à jour (Hz)
        }],
    )

    # ===== DÉTECTION D'OBSTACLES =====
    
    # Nœud pour calculer la distance entre les parois et l'effecteur
    # Évite les collisions avec les murs et obstacles fixes
    distance_parois_node = Node(
        package='franka_teleop',
        executable='distance_parois',
        name='distance_parois_node',
        output='screen'
    )

    # Nœud pour calculer la distance entre l'objet et l'effecteur
    # Aide à la préhension et manipulation d'objets
    distance_objet_node = Node(
        package='franka_teleop',
        executable='distance_objet',
        name='distance_objet_node',
        output='screen'
    )

    # ===== GUIDAGE VIRTUEL =====
    
    # Nœud pour le guidage virtuel
    # Fournit une assistance à l'opérateur pour guider le robot
    guide_virtuel_node = Node(
        package='franka_teleop',
        executable='guide_virtuel',
        name='guide_virtuel_node',
        output='screen'
    )

    # ===== CONSTRUCTION DE LA DESCRIPTION DE LANCEMENT =====
    return LaunchDescription([
        # === ARGUMENTS DE LANCEMENT ===
        
        # Fichier de configuration des robots
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_bringup'), 'config', 'franka.config.yaml'
            ]),
            description='Chemin vers le fichier de configuration YAML des robots',
        ),
        
        # Nom du contrôleur principal à lancer
        DeclareLaunchArgument(
            'controller_name',
            description='Nom du contrôleur principal à spawner (obligatoire)',
        ),
        
        # Mode simulation pour la synchronisation temporelle
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='false',
            description='Utilise l\'horloge de simulation (Gazebo) si vrai'
        ),
        
        # Nom du contrôleur du gripper
        DeclareLaunchArgument(
            'gripper_example_controller',
            default_value='gripper_example_controller',
            description='Nom du contrôleur du gripper à spawner',
        ),
        
        # Nom du contrôleur de vitesse cartésienne
        DeclareLaunchArgument(
            'cartesian_velocity_example_controller',
            default_value='cartesian_velocity_example_controller',
            description='Nom du contrôleur de vitesse cartésienne à spawner',
        ),
        
        # === NŒUDS DU SYSTÈME ===
        
        # Fonction pour générer les nœuds des robots
        OpaqueFunction(function=generate_robot_nodes),
        
        # Nœuds de traitement des données
        offset_position,
        value_to_speed,
        
        # Nœuds de retour de force haptique
        force_vitesse_node,
        force_position_node,
        
        # Nœuds de calculs cinématiques
        ig_solver_node,
        mgd_node,
        
        # Nœud de gestion des modes
        controller_switcher_node,
        
        # Nœuds de surveillance et guidage
        fiole_pose_monitor,
        distance_parois_node,
        distance_objet_node,
        guide_virtuel_node,
    ])
