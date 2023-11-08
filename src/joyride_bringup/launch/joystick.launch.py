# Python
import os
from ament_index_python.packages import get_package_share_directory

# ROS
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    return LaunchDescription([

        # Transforms
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_core'), 'launch'),
            '/static_transforms.launch.py'
            ])
        ),

        # Vel Preprocess
        Node(
            package='joyride_control_py',
            executable='vel_preprocessor',
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