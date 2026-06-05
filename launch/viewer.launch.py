import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("subwoofer_model"),
            "rviz",
            "subwoofer.rviz"
        )]
    )


    return LaunchDescription([
        rviz_node
    ])
