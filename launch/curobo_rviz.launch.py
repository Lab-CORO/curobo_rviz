import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    max_attempts = LaunchConfiguration('max_attempts')
    timeout = LaunchConfiguration('timeout')
    time_dilation_factor = LaunchConfiguration('time_dilation_factor')
    voxel_size = LaunchConfiguration('voxel_size')
    collision_activation_distance = LaunchConfiguration('collision_activation_distance')
    # Création du nœud rviz2 avec les paramètres récupérés des arguments de lancement
    start_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('curobo_rviz'), 'rviz2', 'config.rviz')],
        parameters=[{
            'max_attempts': max_attempts,
            'timeout': timeout,
            'time_dilation_factor': time_dilation_factor,
            'voxel_size': voxel_size,
            'collision_activation_distance': collision_activation_distance
        }]
    )

    # Création de la description de lancement
    ld = LaunchDescription()

    # Ajout du nœud rviz2
    ld.add_action(start_rviz2)

    return ld
