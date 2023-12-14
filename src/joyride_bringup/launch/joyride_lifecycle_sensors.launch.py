# Created: Spring 2023, comments added 12/13/23 by Josephine Wade
# This was a work in progress file to create working lifecycle nodes for each sensor
# Not sure if the blackfly camera lifecycles were finished - testing required 
# Possibly move 2D lidar lifecycle node here (?)

# Python imports
from http.server import executable
import os
from ament_index_python.packages import get_package_share_directory

# ROS imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch.action import DeclareLaunchArgument
#from launch.substitutions import TextSubstitution


def generate_launch_description():

    return LaunchDescription([

        # Cameras
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('blackfly_camera_driver'), 'launch'),
                '/bfly_no_lifecycle.launch.py'
        ])
        )

        
    ])