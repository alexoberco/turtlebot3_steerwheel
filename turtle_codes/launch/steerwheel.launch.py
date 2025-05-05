#!/usr/bin/env python3

from launch import LaunchDescription                           # API de lanzamiento en Python :contentReference[oaicite:2]{index=2}
from launch_ros.actions import Node                            # Acción para ejecutar nodos ROS :contentReference[oaicite:3]{index=3}

def generate_launch_description():
    return LaunchDescription([
        # 1) Arranca joy_node para /dev/input/js0
        Node(
            package='joy',                                     
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[                                       
                {'dev': '/dev/input/js0'},
            ]
        ),
        # 2) Arranca tu nodo turtlebot_steerwheel
        Node(
            package='turtle_codes',                       
            executable='turtlebot_steerwheel',                 
            name='steer_wheel',
            output='screen',
            # remapea si fuera necesario (aquí no hay remappings)
        ),
    ])
