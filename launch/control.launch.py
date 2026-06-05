from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    controller_arg = DeclareLaunchArgument(
        "launch_controller",
        default_value="true")
    
    joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_controller",
        parameters=[{
            "device_name": "Nintendo Switch Combined Joy-Cons"
        }],
        condition=IfCondition(LaunchConfiguration("launch_controller"))
    )

    sub_controller = Node(
        package="subwoofer",
        executable="remote_controller",
        name="remote_controller"
    )


    return LaunchDescription([
        controller_arg,
        joy,
        sub_controller
    ])
