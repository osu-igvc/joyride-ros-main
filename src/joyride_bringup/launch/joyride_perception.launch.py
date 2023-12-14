# Comments added 12/13/23 by Josephine Wade
# This file was started to add in perception algorithms to launch
# Requires:
#   - Minimal or minimal fake (sensors need to be launched before algorithms can publish anything)

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
        #Blob/orange detector
        Node(
            package="joyride_perception",
            executable="orange_detector",
            name="orange_detector",
        ),
        # Lane detection slider - allows for the image contrast to be adjusted
        Node(
            package='joyride_perception_cpp',
            executable='lane_slider_node',
            namespace='perception',
            name='slider'
        ),

        # Run Lane Detection Node
        Node(
            package='joyride_perception_cpp',  # Package our node is in
            executable='white_detection_node', # Node we want to run
            namespace='perception',
            name='White_Detection'
        ),
        # Run UV to Pointcloud - not sure if this is added in correctly; test on car
        Node(
            package='joyride_perception',  # Package our node is in
            executable='uv_to_pointcloud', # Node we want to run
            name='uv_to_pointcloud'
        ),

        # Add in YOLO Sign detection
        
        # Add in pothole detection
        
    ])