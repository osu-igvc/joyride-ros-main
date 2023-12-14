# Created Fall 2023, comments added 12/14/23
# This is a launch file to run OctoMap; most of the config has been left to default values
# Octomap takes in pointcloud data only and creates a occupancy grid from the sensor data; does not clear out objects yet
# Could be used to generate the inital global map with lanes 

import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[
                {'resolution': 0.05},
                {'frame_id': 'local_enu'},
                {'sensor_model/max_range': '15.0'}
            ],
            remappings=[
                ('cloud_in', '/perception/center/lanes_point_cloud')
            ]
        )
    ])

