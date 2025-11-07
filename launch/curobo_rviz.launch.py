import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Déclaration des arguments de lancement avec valeurs par défaut
    declare_max_attempts = DeclareLaunchArgument(
        'max_attempts',
        default_value='10',
        description='Maximum number of attempts'
    )

    declare_timeout = DeclareLaunchArgument(
        'timeout',
        default_value='5.0',
        description='Timeout value'
    )

    declare_time_dilation_factor = DeclareLaunchArgument(
        'time_dilation_factor',
        default_value='1.0',
        description='Time dilation factor'
    )

    declare_voxel_size = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.05',
        description='Voxel size'
    )

    declare_collision_activation_distance = DeclareLaunchArgument(
        'collision_activation_distance',
        default_value='0.1',
        description='Collision activation distance'
    )

    declare_base_link = DeclareLaunchArgument(
        'base_link',
        default_value='base_0',
        description='Base link frame name'
    )

    max_attempts = LaunchConfiguration('max_attempts')
    timeout = LaunchConfiguration('timeout')
    time_dilation_factor = LaunchConfiguration('time_dilation_factor')
    voxel_size = LaunchConfiguration('voxel_size')
    collision_activation_distance = LaunchConfiguration('collision_activation_distance')
    base_link = LaunchConfiguration('base_link')

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
            'collision_activation_distance': collision_activation_distance,
            'base_link': base_link
        }]
    )

    # Création de la description de lancement
    ld = LaunchDescription()

    # Ajout des déclarations d'arguments
    ld.add_action(declare_max_attempts)
    ld.add_action(declare_timeout)
    ld.add_action(declare_time_dilation_factor)
    ld.add_action(declare_voxel_size)
    ld.add_action(declare_collision_activation_distance)
    ld.add_action(declare_base_link)

    # Ajout du nœud rviz2
    ld.add_action(start_rviz2)

    return ld
