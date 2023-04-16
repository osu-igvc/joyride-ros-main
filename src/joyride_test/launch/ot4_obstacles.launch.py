# Launch file for outdoor test 4, camera data gathering.

# Python
from http.server import executable
import os
from ament_index_python.packages import get_package_share_directory

# ROS
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bag_name = 'camera_test'

    obstacle_detect_config = os.path.join(
        get_package_share_directory('joyride_bringup'),
        'config',
        'obstacle_config.yaml'
    )


    return LaunchDescription([
        # Bagger
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('joyride_bringup'), 'launch'),
                '/data_log_bringup.launch.py'
        ]),
        launch_arguments={'bag_name': bag_name}.items()
        ),


        # Cameras
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('blackfly_camera_driver'), 'launch'),
                '/bfly_center_lifecycle.launch.py'
        ])
        ),

        # Obstacle detection
        Node(
            package='joyride_perception',
            executable='blob_detector',
            namespace='perception',
            name='pedestrian_detector',
            parameters=[obstacle_detect_config]
        ),

        # Pedestrian detection
        Node(
            package='joyride_perception',
            executable='obstacle_detector',
            namespace='perception',
            name='obstacle_detector',
            parameters=[obstacle_detect_config]
        )


    ])