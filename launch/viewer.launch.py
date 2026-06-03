import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory("subwoofer_model"),
        "model",
        "subwoofer.urdf.xacro"
    )

    robot_description = xacro.process_file(xacro_file).toxml()

    



    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description
        }]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("subwoofer_model"),
            "rviz",
            "urdf.rviz"
        )]
    )


    return LaunchDescription([
        rsp_node,
        rviz_node
    ])
