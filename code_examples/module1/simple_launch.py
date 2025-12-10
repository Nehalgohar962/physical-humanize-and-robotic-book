from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates a launch description to start the talker and listener nodes.
    """
    return LaunchDescription([
        Node(
            package='my_first_package', # Replace with your actual package name
            executable='talker_node',    # The name of the executable from setup.py
            name='my_talker'
        ),
        Node(
            package='my_first_package', # Replace with your actual package name
            executable='listener_node',  # The name of the executable from setup.py
            name='my_listener'
        ),
    ])
