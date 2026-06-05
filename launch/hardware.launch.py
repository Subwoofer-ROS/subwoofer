import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    simulated_arg = DeclareLaunchArgument(
        "simulated",
        default_value="true"
    )
    
    camera_arg = DeclareLaunchArgument(
        "camera",
        default_value="true"
    )



    xacro_file = os.path.join(
        get_package_share_directory("subwoofer_model"),
        "model",
        "subwoofer.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description
        }]
    )
    
    hardware_node = Node(
        package="subwoofer",
        executable="subwoofer",
        name="subwoofer",
        parameters=[{
            "simulated": LaunchConfiguration("simulated")
        }]
    )

    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        parameters=[{
            "camera_frame_id": "camera_link"
        }],
        condition=IfCondition(LaunchConfiguration("camera"))
    )



    


    return LaunchDescription([
        simulated_arg,
        camera_arg,

        rsp_node,
        hardware_node,
        camera_node,
    ])
