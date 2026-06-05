from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    state_manager = Node(
        package="subwoofer",
        executable="state_manager",
        name="state_manager"
    )

    trot_controller = Node(
        package="subwoofer",
        executable="trot_controller",
        name="trot_controller",
        namespace="controllers"
    )

    stance_controller = Node(
        package="subwoofer",
        executable="stance_controller",
        name="stance_controller",
        namespace="controllers"
    )

    


    return LaunchDescription([
        state_manager,
        trot_controller,
        stance_controller
    ])
