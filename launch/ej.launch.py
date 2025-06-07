#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declarar argumento para el texto
    text_arg = DeclareLaunchArgument(
        'text',
        default_value='Hola mundo desde ROS2',
        description='Texto a procesar por el action server'
    )

    # Nodo servidor
    server_node = Node(
        package='kruger_guariste_ej_2',
        executable='text_processor_server',
        name='text_processor_server',
        output='screen'
    )

    # Nodo cliente
    client_node = Node(  
        package='kruger_guariste_ej_2',
        executable='text_processor_client',
        name='text_processor_client',
        output='screen',
        arguments=[LaunchConfiguration('text')]
    )

    return LaunchDescription([
        text_arg,
        server_node,
        client_node,
    ])