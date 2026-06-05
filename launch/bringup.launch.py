from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    simulated_arg = DeclareLaunchArgument(
        "simulated",
        default_value="true")
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false")
    camera_arg = DeclareLaunchArgument(
        "camera",
        default_value="true")
    controller_arg = DeclareLaunchArgument(
        "launch_controller",
        default_value="true")
    
    hardware_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("subwoofer"),
            "launch",
            "hardware.launch.py"
        ]),
        launch_arguments={
            "simulated": LaunchConfiguration("simulated"),
            "camera": LaunchConfiguration("camera")
        }.items()
    )

    control_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("subwoofer"),
            "launch",
            "control.launch.py"
        ]),
        launch_arguments={
            "launch_controller": LaunchConfiguration("launch_controller")
        }
    )

    behaviour_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("subwoofer"),
            "launch",
            "behaviour.launch.py"
        ])
    )


    return LaunchDescription([
        simulated_arg,
        rviz_arg,
        camera_arg,
        controller_arg,

        hardware_launch,
        control_launch,
        behaviour_launch
    ])
