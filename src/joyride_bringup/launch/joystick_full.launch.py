# Python
from http.server import executable
import os
from ament_index_python.packages import get_package_share_directory

# ROS
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    




    return LaunchDescription([

        # Cameras
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #         get_package_share_directory('blackfly_camera_driver'), 'launch'),
        #         '/bfly_lifecycle.launch.py'
        # ])
        # ),

        # CAN Server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_core'), 'launch'),
            '/can_server_bringup.launch.py'
            ])
         ),

        # ROSBAGGER
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/data_log_bringup.launch.py'
            ])
        ),

        # Transforms
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_core'), 'launch'),
            '/static_transforms.launch.py'
            ])
        ),

        # Vectornav
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #     get_package_share_directory('vectornav'), 'launch'),
        #     '/vectornav.launch.py'
        #     ])
        # ),

        # Vel Preprocess
        Node(
            package='joyride_control',
            executable='vel_preprocess_node',
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