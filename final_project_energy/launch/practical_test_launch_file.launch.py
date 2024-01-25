from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="final_project_energy", executable="show_energy", output="screen"),
        ]
    )