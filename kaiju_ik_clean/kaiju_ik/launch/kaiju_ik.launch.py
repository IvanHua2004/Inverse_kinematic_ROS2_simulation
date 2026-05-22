from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='kaiju_ik',
            executable='kaiju_ik_node',
            name='kaiju_ik',
            output='screen'
        ),
        Node(
            package='kaiju_ik',
            executable='target_mover',
            name='target_spawner',
            output='screen'
        )
    ])
