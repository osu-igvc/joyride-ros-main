# Created: Spring 2023, comments added 12/13/23 by Josephine Wade
# 
# File launches:
#   - minimal (static Tfs, Diagnostics, sensors, localization, and velocity preprocessor)
#   - joystick control software
# Once launched, Car needs to go into AutoMode through the request button on the GUI 
# After car has entered Autonomy Mode the joystick will be able to control the car 

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


        # Minimal
        IncludeLaunchDescription(
            
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/joyride_minimal.launch.py'
            ])
        ),

        # Joystick
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_hmi'), 'launch'),
            '/joystick_bringup.launch.py'
        ])
    )
    ])
