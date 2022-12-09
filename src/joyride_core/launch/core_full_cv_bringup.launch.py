# Python
from http.server import executable
import os
from ament_index_python.packages import get_package_share_directory

# ROS
from launch import LaunchDescription
from launch_ros.actions import Node
#from launch.action import DeclareLaunchArgument
#from launch.substitutions import TextSubstitution


def generate_launch_description():

    vision_config = os.path.join(
        get_package_share_directory('joyride_core'),
        'config',
        'vision_config.yaml'
    )

    return LaunchDescription([

        # ----------- Create Pedestrian (Orange blob) Detector ----------- #
        Node(
            package='joyride_perception',
            namespace='perception',
            executable='blob_detector',
            name='pedestrian_detector_node',
            parameters=[vision_config]
        ),

        # ----------- Publish Lidar Streams ----------- #

    ])