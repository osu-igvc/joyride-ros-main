
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    bag_name = 'remote_control'

    obstacle_detect_config = os.path.join(
        get_package_share_directory('joyride_bringup'),
        'config',
        'obstacle_config.yaml'
    )

    return LaunchDescription([
        # Minimal
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('joyride_bringup'), 'launch'),
                '/joyride_minimal.launch.py'
            ])
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
        ),
        # GPS/IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('vectornav'), 'launch'),
                '/vectornav.launch.py'
        ])
        ),
        # 2D Lidar
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('joyride_bringup'), 'launch'),
                '/lidar2D.launch.py'
        ])
        )
    ])
