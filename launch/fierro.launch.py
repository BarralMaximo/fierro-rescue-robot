"""Levanta los tres nodos de Fierro. No mueven al robot hasta recibir el mapa en /map.

    ros2 launch fierro fierro.launch.py victims:=N

`victims` es la cantidad de víctimas a buscar. `use_sim_time` va en true solo en el simulador: el robot real no publica
/clock.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription

def generate_launch_description():
    params = os.path.join(get_package_share_directory('fierro'), 'params', 'params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    victims = LaunchConfiguration('victims')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('victims', default_value='5',
                              description='cantidad de víctimas a buscar (N)'),
        Node(
            package='fierro',
            executable='perception',
            name='perception',
            output='screen',
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'victims': ParameterValue(victims, value_type=int),
            }],
        ),
        Node(
            package='fierro',
            executable='localizer',
            name='localizer',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='fierro',
            executable='navigator',
            name='navigator',
            output='screen',
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'victims': ParameterValue(victims, value_type=int),
            }],
        ),
    ])
