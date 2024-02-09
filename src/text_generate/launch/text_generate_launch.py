from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="text_generate", executable="talker", name="talker"),
            Node(
                package="text_generate", executable="asr_listener", name="asr_listener"
            ),
            Node(
                package="text_generate", executable="obj_listener", name="obj_listener"
            ),
            Node(
                package="text_generate", executable="per_listener", name="per_listener"
            ),
        ]
    )
