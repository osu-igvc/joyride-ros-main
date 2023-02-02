# Python
from http.server import executable
import os
from ament_index_python.packages import get_package_share_directory

# ROS
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch.action import DeclareLaunchArgument
#from launch.substitutions import TextSubstitution


def generate_launch_description():

    vision_config = os.path.join(
        get_package_share_directory('joyride_core'),
        'config',
        'vision_config.yaml'
    )


    # ----------- Launch Cameras ----------- #
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_core'), 'launch'),
            '/sensor_bringup.launch.py'
        ])
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

         # ----------- White Lane Detector ----------- #
        Node(
            package='joyride_perception',
            namespace='perception',
            executable='lane_detector',
            name='lane_detection_node',
            parameters=[vision_config]
        ),

        # ----------- Launch Cameras ----------- #
        sensor_launch
    ])