from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import Command
import os
import xacro


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory("subwoofer_model"),
        "urdf",
        "subwoofer.urdf.xacro"
    )

    robot_description = xacro.process_file(xacro_file).toxml()

    

    robot_node = Node(
        package="subwoofer",
        executable="subwoofer",
        name="subwoofer"
    )

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
        robot_node,
        rsp_node,
        rviz_node
    ])
