from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Iniciar turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # Iniciar nuestro controlador
        Node(
            package='g02_prii3_turtlesim',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen'
        )
    ])
