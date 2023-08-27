from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'topic_subscriber_pkg',
            executable = 'simple_subscriber_node',
            output = 'screen',
            emulate_tty= True),
])