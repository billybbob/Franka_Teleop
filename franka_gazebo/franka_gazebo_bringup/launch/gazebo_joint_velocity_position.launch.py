import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file, 
        mappings={
            'arm_id': arm_id_str, 
            'hand': load_gripper_str, 
            'ros2_control': 'true', 
            'gazebo': 'false', 
            'ee_id': franka_hand_str
        }
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ]
    )

    return [robot_state_publisher]

def generate_launch_description():
    # Configure ROS nodes for launch
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)

    # Ajout de paramètres pour le pont ROS-Gazebo
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    log_level = LaunchConfiguration('log_level', default='info')

    load_gripper_launch_argument = DeclareLaunchArgument(
            load_gripper_name,
            default_value='false',
            description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
            franka_hand_name,
            default_value='franka_hand',
            description='Default value: franka_hand')
    arm_id_launch_argument = DeclareLaunchArgument(
            arm_id_name,
            default_value='fr3',
            description='Available values: fr3, fp3 and fer')

    # Get robot description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand])

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Définir le chemin vers votre monde personnalisé
    custom_world_path = os.path.join(get_package_share_directory('ros_gz_example_gazebo'), 'worlds', 'franka.sdf')

    # Lancer Gazebo avec votre monde personnalisé
    gazebo_custom_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{custom_world_path} -r', }.items(),
    )

    # Spawn dans gazebo avec les positions initiales spécifiées
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
        '-topic', '/robot_description',
        '-x', '0.0',   # Position X
        '-y', '-0.5',   # Position Y
        '-z', '0.5',   # Position Z
        '-R', '-1.57',   # Roll
        '-P', '0.0',   # Pitch
        '-Y', '0.0',   # Yaw
        ],
        output='screen',
    )

    # Visualize in RViz
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file, '-f', 'world'],
    )
    
    # Chargement des contrôleurs dans le bon ordre
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output='screen'
    )

    # Charger le contrôleur de position en état actif
    joint_position_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
                'joint_position_example_controller'],
        output='screen'
    )

    # Charger le contrôleur de vitesse en état inactif
    joint_velocity_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
                'joint_velocity_example_controller'],
        output='screen'
    )

    # Charger le contrôleur de la pince en état actif
    gripper_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
                'gripper_example_controller'],
        output='screen'
    )

    # Charger les contrôleurs cartésiens en état inactif
    cartesian_velocity_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
                'cartesian_velocity_example_controller'],
        output='screen'
    )

    cartesian_pose_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
                'cartesian_pose_example_controller'],
        output='screen'
    )

    # Nœud pour lancer le solveur IK
    ik_solver_node = Node(
        package='franka_teleop',      # Nom du package contenant le script
        executable='franka_ik_solver', # Nom de l'exécutable
        name='franka_ik_solver',      # Nom du nœud
        output='screen'               # Afficher la sortie à l'écran
    )

    # Nœud pour lancer le solveur IG
    ig_solver_node = Node(
        package='franka_teleop',      # Nom du package contenant le script
        executable='franka_ig_solver', # Nom de l'exécutable
        name='franka_ig_solver',      # Nom du nœud
        output='screen'               # Afficher la sortie à l'écran
    )

    # Nœud pour lancer le MGD
    mgd_node = Node(
        package='franka_teleop',        # Nom du package contenant le script
        executable='mgd',               # Nom de l'exécutable
        name='mgd_node',                # Nom du nœud
        output='screen'                 # Afficher la sortie à l'écran
    )

    # Nœud pour lancer le noeud qui permet de changer de mode
    controller_switcher_node = Node(
        package='franka_teleop',
        executable='switch_mode',
        name='controller_switcher',
        output='screen'
    )

    # Nœud pour lancer le force_position
    force_position_node = Node(
        package='franka_teleop',        # Nom du package contenant le script
        executable='force_position',               # Nom de l'exécutable
        name='force_position_node',                # Nom du nœud
        output='screen'                 # Afficher la sortie à l'écran
    )

    # Nœud pour lancer le force_vitesse
    force_vitesse_node = Node(
        package='franka_teleop',        # Nom du package contenant le script
        executable='force_vitesse',               # Nom de l'exécutable
        name='force_vitesse_node',                # Nom du nœud
        output='screen'                 # Afficher la sortie à l'écran
    )

    # Nœud pour connaitre la position de la fiole
    fiole_pose_monitor = Node(
        package="franka_teleop",
        executable="fiole_pose_monitor",
        name="fiole_pose_monitor",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "object_name": "Fiole_world", "poll_rate": 10.0}],
    )

    # Nœud pour connaitre la distance entre les parroies et l'effecteur du robot
    distance_parois_node = Node(
        package='franka_teleop',        # Nom du package contenant le script
        executable='distance_parois',               # Nom de l'exécutable
        name='distance_parois_node',                # Nom du nœud
        output='screen'                 # Afficher la sortie à l'écran
    )

    # Nœud pour lancer guide_virtuel
    guide_virtuel_node = Node(
        package='franka_teleop',        # Nom du package contenant le script
        executable='guide_virtuel',               # Nom de l'exécutable
        name='guide_virtuel_node',                # Nom du nœud
        output='screen'                 # Afficher la sortie à l'écran
    )

    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        DeclareLaunchArgument('use_sim_time', default_value='true',
                             description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('log_level', default_value='info',
                             description='Log level for the bridge'),
        gazebo_custom_world,
        robot_state_publisher,
        rviz,
        spawn,
        RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn,
                    on_exit=[load_joint_state_broadcaster],
                )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[gripper_example_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[joint_position_example_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[joint_velocity_example_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[cartesian_velocity_example_controller],
            )
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['joint_states'],
                 'rate': 30}],
        ),
        ik_solver_node,
        ig_solver_node,
        mgd_node,
        controller_switcher_node,
        force_position_node,
        force_vitesse_node,
        fiole_pose_monitor,
        distance_parois_node,
        guide_virtuel_node,
    ])
