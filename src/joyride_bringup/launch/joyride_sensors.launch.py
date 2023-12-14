# Comments added 12/13/23 by Josephine Wade
# This file launches all sensors; will need to be modified when more sensors are added
# This file could also be reworked to have launch configurations to decide which sensors could be launched but for now each section can be commented out or brought back in
# Requires:
#   - Sensors to be plugged in and configured (Lidar IP set and camera IPs set manually if IPs not automatically)            

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


    return LaunchDescription([

        # Cameras
        IncludeLaunchDescription(
           PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('blackfly_camera_driver'), 'launch'),
               '/bfly_center_lifecycle.launch.py'])
        ),

        # Vectornav INS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('vectornav'), 'launch'),
                '/vectornav.launch.py'])
        ),

        # 2D Lidar
        IncludeLaunchDescription(
           PythonLaunchDescriptionSource([os.path.join(
               get_package_share_directory('joyride_bringup'), 'launch'),
               '/joyride_lidar2D.launch.py'])
        ),

        
    ])
