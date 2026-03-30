# Launch file for face detection

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cascade_path = os.path.join(get_package_share_directory('subwoofer'), 'statics', 'haarcascade_frontalface_default.xml')

    return LaunchDescription([
        Node(
            package='subwoofer',
            executable='FaceDetection',
            name='face_detection',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'cascade_path': cascade_path}]
        )
    ])