from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates a launch description to start the full VLA system.
    """
    return LaunchDescription([
        Node(
            package='my_vla_package',     # Replace with your actual package name
            executable='whisper_node',
            name='whisper'
        ),
        Node(
            package='my_vla_package',     # Replace with your actual package name
            executable='llm_commander_node',
            name='llm_commander'
        ),
        Node(
            package='my_vla_package',     # Replace with your actual package name
            executable='detection_node',
            name='detector'
        ),
    ])
