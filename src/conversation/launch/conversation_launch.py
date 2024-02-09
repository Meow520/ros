from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="conversation", executable="talk", name="talk"),
            Node(
                package="conversation", executable="asr_listener", name="asr_listener"
            ),
        ]
    )
