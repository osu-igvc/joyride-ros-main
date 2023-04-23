
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


    return LaunchDescription([

        # ROSBAGGER
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/rosbag_recorder.launch.py'
            ]),
            launch_arguments={'bag_name':bag_name}.items()
        ),

        # Minimal
        IncludeLaunchDescription(
            
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/joyride_minimal.launch.py'
            ])
        ),

        # Vel Preprocess
        Node(
            package='joyride_control',
            executable='velocity_preprocessor',
            name='vel_node',
        ),

        # Joystick
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_hmi'), 'launch'),
            '/joystick_bringup.launch.py'
        ])
    )
    ])
