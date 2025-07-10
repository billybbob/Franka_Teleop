#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Arguments de lancement optionnels
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Niveau de log pour tous les nœuds'
        ),
        
        # Message d'information au démarrage
        LogInfo(msg='Lancement des deux nœuds du projet test_cartesien...'),
        
        # Nœud value_to_speed
        Node(
            package='test_cartesien',
            executable='value_to_speed',
            name='value_to_speed',
            output='screen',
            parameters=[
                {'log_level': LaunchConfiguration('log_level')}
            ],
        ),
        
        # Nœud offset_position
        Node(
            package='test_cartesien',
            executable='offset_position_simu',
            name='offset_position',
            output='screen',
            parameters=[
                {'log_level': LaunchConfiguration('log_level')}
            ],
        ),
        
        LogInfo(msg='Tous les nœuds ont été lancés avec succès!')
    ])
