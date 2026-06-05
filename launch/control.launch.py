from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_controller",
        parameters=[{
            "device_name": "Nintendo Switch Combined Joy-Cons"
        }]
    )

    sub_controller = Node(
        package="subwoofer",
        executable="remote_controller",
        name="remote_controller"
    )


    return LaunchDescription([
        joy,
        sub_controller
    ])
