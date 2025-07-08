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

# Add the path to the `utils` folder
package_share = get_package_share_directory('franka_bringup')
utils_path = os.path.join(package_share, '..', '..', 'lib', 'franka_bringup', 'utils')
sys.path.append(os.path.abspath(utils_path))

from launch_utils import load_yaml  # noqa: E402


def generate_robot_nodes(context):
    config_file = LaunchConfiguration('robot_config_file').perform(context)
    controller_name = LaunchConfiguration('controller_name').perform(context)
    gripper_controller_name = LaunchConfiguration('gripper_example_controller').perform(context)
    cartesian_velocity_controller_name = LaunchConfiguration('cartesian_velocity_example_controller').perform(context)
    configs = load_yaml(config_file)
    nodes = []

    for item_name, config in configs.items():
        namespace = config['namespace']
        
        # Include base robot setup
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'
                    ])
                ),
                launch_arguments={
                    'arm_id': str(config['arm_id']),
                    'arm_prefix': str(config['arm_prefix']),
                    'namespace': str(namespace),
                    'urdf_file': str(config['urdf_file']),
                    'robot_ip': str(config['robot_ip']),
                    'load_gripper': str(config['load_gripper']),
                    'use_fake_hardware': str(config['use_fake_hardware']),
                    'fake_sensor_commands': str(config['fake_sensor_commands']),
                    'joint_sources': ','.join(config['joint_sources']),
                    'joint_state_rate': str(config['joint_state_rate']),
                }.items(),
            )
        )

        # Spawner for main controller
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
        
        # Spawner for gripper controller
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
        
        # Spawner for cartesian velocity controller
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

    # Optional: RViz if use_rviz is set
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

    # Ajout de paramètres pour le pont ROS-Gazebo
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Nœud pour lancer le force_vitesse
    force_vitesse_node = Node(
        package='franka_teleop',
        executable='force_vitesse',
        name='force_vitesse_node',
        output='screen'
    )

    # Nœud pour lancer le force_position
    force_position_node = Node(
        package='franka_teleop',        # Nom du package contenant le script
        executable='force_position',               # Nom de l'exécutable
        name='force_position_node',                # Nom du nœud
        output='screen'                 # Afficher la sortie à l'écran
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

    # Nœud pour connaitre la distance entre l'objet et l'effecteur du robot
    distance_objet_node = Node(
        package='franka_teleop',        # Nom du package contenant le script
        executable='distance_objet',               # Nom de l'exécutable
        name='distance_objet_node',                # Nom du nœud
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
        DeclareLaunchArgument(
            'robot_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_bringup'), 'config', 'franka.config.yaml'
            ]),
            description='Path to the robot configuration file to load',
        ),
        DeclareLaunchArgument(
            'controller_name',
            description='Name of the controller to spawn (required, no default)',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'gripper_example_controller',
            default_value='gripper_example_controller',
            description='Name of the gripper controller to spawn',
        ),
        DeclareLaunchArgument(
            'cartesian_velocity_example_controller',
            default_value='cartesian_velocity_example_controller',
            description='Name of the cartesian velocity controller to spawn',
        ),
        OpaqueFunction(function=generate_robot_nodes),
        force_vitesse_node,
        force_position_node,
        ig_solver_node,
        mgd_node,
        controller_switcher_node,
        fiole_pose_monitor,
        distance_parois_node,
        distance_objet_node,
        guide_virtuel_node,
    ])
