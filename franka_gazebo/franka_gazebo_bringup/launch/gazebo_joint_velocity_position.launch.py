#!/usr/bin/env python3
"""
Fichier de lancement ROS2 pour la téléopération d'un robot Franka FR3 dans Gazebo Fortress.

Ce fichier configure et lance un environnement de simulation complet pour la téléopération
d'un robot Franka FR3 avec un contrôleur haptique Desktop 6D de Haption. Il initialise
Gazebo avec un monde personnalisé, charge les contrôleurs de position et vitesse,
et démarre tous les nœuds nécessaires pour la téléopération.

Auteur: Vincent Bassemayousse
Date: 07/10/2025
Version: 1.0
Licence: Apache 2.0
Prérequis: ROS2 Humble, Gazebo Fortress, franka_description, franka_teleop packages
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node


def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    """
    Génère la description URDF du robot Franka à partir des fichiers XACRO.
    
    Cette fonction traite les fichiers XACRO pour créer une description URDF complète
    du robot Franka, incluant les paramètres pour la simulation Gazebo et ros2_control.
    
    Args:
        context (LaunchContext): Contexte de lancement pour la substitution des variables
        arm_id (LaunchConfiguration): ID du bras robotique (fr3, fp3, fer)
        load_gripper (LaunchConfiguration): Active/désactive la pince (true/false)
        franka_hand (LaunchConfiguration): Type de pince utilisée
    
    Returns:
        list: Liste contenant le nœud robot_state_publisher configuré
    
    Raises:
        FileNotFoundError: Si le fichier XACRO du robot n'est pas trouvé
        XacroException: Si le traitement XACRO échoue
    """
    # Substitution des variables de lancement dans le contexte actuel
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    # Construction du chemin vers le fichier XACRO du robot
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    # Traitement du fichier XACRO avec les paramètres spécifiques
    robot_description_config = xacro.process_file(
        franka_xacro_file, 
        mappings={
            'arm_id': arm_id_str,           # Identifiant du bras
            'hand': load_gripper_str,       # Chargement de la pince
            'ros2_control': 'true',         # Activation de ros2_control
            'gazebo': 'true',               # Configuration pour Gazebo
            'ee_id': franka_hand_str        # Identifiant de l'effecteur terminal
        }
    )
    
    # Conversion en dictionnaire pour robot_state_publisher
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Configuration du nœud robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',  # Affichage sur stdout et stderr
        parameters=[robot_description]
    )

    return [robot_state_publisher]


def generate_launch_description():
    """
    Génère la description de lancement complète pour la téléopération Franka.
    
    Cette fonction configure l'environnement de simulation complet incluant:
    - Gazebo avec monde personnalisé
    - Robot Franka avec contrôleurs position/vitesse
    - Nœuds de téléopération et de traitement
    - Systèmes de guidage virtuel et de forces
    - Monitoring des objets et distances
    
    Returns:
        LaunchDescription: Description complète du lancement ROS2
    """
    
    # ==================== CONFIGURATION DES PARAMÈTRES ====================
    
    # Noms des paramètres de configuration du robot
    load_gripper_name = 'load_gripper'    # Paramètre pour activer la pince
    franka_hand_name = 'franka_hand'      # Type de pince Franka
    arm_id_name = 'arm_id'                # Modèle du bras robotique

    # Configuration des variables de lancement
    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)

    # Paramètres pour la simulation et le logging
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    log_level = LaunchConfiguration('log_level', default='info')

    # ==================== ARGUMENTS DE LANCEMENT ====================
    
    # Argument pour activer/désactiver la pince
    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name,
        default_value='false',
        description='Active (true) ou désactive (false) la pince du robot'
    )
    
    # Argument pour le type de pince
    franka_hand_launch_argument = DeclareLaunchArgument(
        franka_hand_name,
        default_value='franka_hand',
        description='Type de pince utilisée (défaut: franka_hand)'
    )
    
    # Argument pour le modèle de bras
    arm_id_launch_argument = DeclareLaunchArgument(
        arm_id_name,
        default_value='fr3',
        description='Modèle du bras robotique (fr3, fp3, fer)'
    )

    # ==================== CONFIGURATION DU ROBOT ====================
    
    # Génération de la description URDF du robot
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand]
    )

    # ==================== CONFIGURATION DE GAZEBO ====================
    
    # Configuration des chemins de ressources pour Gazebo
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description')
    )
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Chemin vers le monde personnalisé de simulation
    custom_world_path = os.path.join(
        get_package_share_directory('ros_gz_example_gazebo'), 
        'worlds', 
        'franka.sdf'
    )

    # Lancement de Gazebo avec le monde personnalisé
    gazebo_custom_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'{custom_world_path} -r'  # -r pour démarrage automatique
        }.items()
    )

    # ==================== SPAWN DU ROBOT ====================
    
    # Spawn du robot dans Gazebo avec position et orientation initiales
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',  # Topic contenant la description URDF
            '-x', '0.0',      # Position X (mètres)
            '-y', '-0.5',     # Position Y (mètres)
            '-z', '0.5',      # Position Z (mètres)
            '-R', '-1.57',    # Rotation Roll (radians)
            '-P', '0.0',      # Rotation Pitch (radians)
            '-Y', '0.0',      # Rotation Yaw (radians)
        ],
        output='screen'
    )
    
    # ==================== CONTRÔLEURS ROS2_CONTROL ====================
    
    # Chargement du broadcster d'état des joints (obligatoire en premier)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Contrôleur de position des joints (actif par défaut)
    joint_position_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'joint_position_example_controller'],
        output='screen'
    )

    # Contrôleur de vitesse des joints (inactif par défaut)
    joint_velocity_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
             'joint_velocity_example_controller'],
        output='screen'
    )

    # Contrôleur de la pince (actif pour la simulation)
    gripper_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'gripper_example_controller_simu'],
        output='screen'
    )

    # ==================== CONFIGURATION DE LA CAMÉRA ====================
    
    # Positionnement automatique de la caméra Gazebo après 5 secondes
    set_camera_pose = TimerAction(
        period=5.0,  # Délai en secondes
        actions=[
            ExecuteProcess(
                cmd=[
                    'ign', 'service', '-s', '/gui/move_to/pose',
                    '--reqtype', 'ignition.msgs.GUICamera',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '2000',
                    '--req',
                    'pose: { position: {x: 0.0, y: 1.75, z: 0.6}, '
                    'orientation: {x: 0, y: 0, z: -1, w: 1} }, '
                    'follow_target: false, '
                    'name: "user_camera"'
                ],
                output='screen'
            )
        ]
    )

    # ==================== NŒUDS DE TÉLÉOPÉRATION ====================
    
    # Nœud pour la gestion des offsets de position
    offset_position = Node(
        package='test_cartesien',
        executable='offset_position_simu',
        name='offset_position',
        output='screen'
    )

    # Nœud pour la conversion valeurs vers vitesses
    value_to_speed = Node(
        package='test_cartesien',
        executable='value_to_speed',
        name='value_to_speed',
        output='screen'
    )

    # ==================== SOLVEURS CINÉMATIQUES ====================
    
    # Solveur de cinématique inverse (IK - Inverse Kinematics)
    ik_solver_node = Node(
        package='franka_teleop',
        executable='franka_ik_solver',
        name='franka_ik_solver',
        output='screen'
    )

    # Solveur de cinématique inverse généralisée (IG - Inverse Generalized)
    ig_solver_node = Node(
        package='franka_teleop',
        executable='franka_ig_solver',
        name='franka_ig_solver',
        output='screen'
    )

    # Modèle géométrique direct (MGD - Forward Kinematics)
    mgd_node = Node(
        package='franka_teleop',
        executable='mgd',
        name='mgd_node',
        output='screen'
    )

    # ==================== GESTION DES CONTRÔLEURS ====================
    
    # Nœud pour changer entre les modes de contrôle (position/vitesse)
    controller_switcher_node = Node(
        package='franka_teleop',
        executable='switch_mode_simu',
        name='controller_switcher',
        output='screen'
    )

    # ==================== RETOUR DE FORCE ====================
    
    # Nœud pour le retour de force en mode position
    force_position_node = Node(
        package='franka_teleop',
        executable='force_position',
        name='force_position_node',
        output='screen'
    )

    # Nœud pour le retour de force en mode vitesse
    force_vitesse_node = Node(
        package='franka_teleop',
        executable='force_vitesse',
        name='force_vitesse_node',
        output='screen'
    )

    # ==================== MONITORING ET PERCEPTION ====================
    
    # Monitoring de la position de la fiole dans la simulation
    fiole_pose_monitor = Node(
        package="franka_teleop",
        executable="fiole_pose_monitor",
        name="fiole_pose_monitor",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "object_name": "Fiole_world",  # Nom de l'objet dans Gazebo
            "poll_rate": 10.0              # Fréquence de polling (Hz)
        }]
    )

    # Calcul de distance entre les parois et l'effecteur terminal
    distance_parois_node = Node(
        package='franka_teleop',
        executable='distance_parois',
        name='distance_parois_node',
        output='screen'
    )

    # Calcul de distance entre l'objet et l'effecteur terminal
    distance_objet_node = Node(
        package='franka_teleop',
        executable='distance_objet',
        name='distance_objet_node',
        output='screen'
    )

    # ==================== GUIDAGE VIRTUEL ====================
    
    # Système de guidage virtuel pour l'assistance à la téléopération
    guide_virtuel_node = Node(
        package='franka_teleop',
        executable='guide_virtuel',
        name='guide_virtuel_node',
        output='screen'
    )

    # ==================== SYSTÈME DE VISION ====================
    
    # Lanceur de caméras pour la vision (commenté pour l'instant)
    camera_launcher_node = Node(
        package='franka_teleop',
        executable='camera_launcher',
        name='camera_launcher_node',
        output='screen',
        parameters=[
            {'world_name': 'demo'},        # Nom du monde Gazebo
            {'model_name': 'Cellule'},     # Nom du modèle de cellule
            {'launch_delay': 2.0}          # Délai de lancement (secondes)
        ]
    )

    # ==================== PUBLISHER D'ÉTAT DES JOINTS ====================
    
    # Publisher des états de joints pour la visualisation
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['joint_states'],  # Sources des états de joints
            'rate': 30                        # Fréquence de publication (Hz)
        }]
    )

    # ==================== CONSTRUCTION DE LA DESCRIPTION DE LANCEMENT ====================
    
    return LaunchDescription([
        # Arguments de lancement
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Utilise l\'horloge de simulation Gazebo si true'
        ),
        DeclareLaunchArgument(
            'log_level', 
            default_value='info',
            description='Niveau de log pour le pont ROS-Gazebo'
        ),
        
        # Lancement de Gazebo et spawn du robot
        gazebo_custom_world,
        robot_state_publisher,
        spawn,
        
        # Gestion séquentielle des contrôleurs avec RegisterEventHandler
        # Le joint_state_broadcaster doit être chargé après le spawn
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster]
            )
        ),
        
        # Les autres contrôleurs après le joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[gripper_example_controller]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[joint_position_example_controller]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[joint_velocity_example_controller]
            )
        ),
        
        # Nœuds de base
        joint_state_publisher,
        
        # Nœuds de téléopération
        offset_position,
        value_to_speed,
        
        # Solveurs cinématiques
        ik_solver_node,
        ig_solver_node,
        mgd_node,
        
        # Gestion des contrôleurs
        controller_switcher_node,
        
        # Monitoring et perception
        fiole_pose_monitor,
        distance_objet_node,
        distance_parois_node,
        
        # Retour de force
        force_position_node,
        force_vitesse_node,
        
        # Guidage virtuel
        guide_virtuel_node,
        
        # Configuration de la caméra
        set_camera_pose,
        
        # Système de vision
        # camera_launcher_node,
    ])
