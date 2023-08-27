from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'topic_publisher_pkg',
            executable = 'avoid_obstacle',
            output = 'screen',
            emulate_tty= True),
])