# Created: Spring 2023, comments added 12/13/23 by Josephine Wade
# A launch file for testing out the Nav2 Outdoor Mapping configuration parameters 
# This file was based on the SimpleGoToPose parameter config that was configured correctly
# Launches:
#   - NavStack 
# Requires:
#   - Minimal to be ran independently and GPS fix to be found before launching
#   - Configured parameter file for all of Nav2 

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

    controller_type = LaunchConfiguration('controller_type')
    nav_params = LaunchConfiguration('nav2_params')

    declare_controller_type_cmd = DeclareLaunchArgument(
        'controller_type',
        default_value='simple_go_to_pose_config.yaml'
    )

    final_nav_params_path = DeclareLaunchArgument(
        'nav2_params',
        default_value=[get_package_share_directory('joyride_bringup'), '/config/','nav2_navigation_params.yaml'])

    return LaunchDescription([
        declare_controller_type_cmd,
        final_nav_params_path,


        # Navstack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/joyride_navstack.launch.py']),
            launch_arguments={'params_file':nav_params}.items()
        ),
        
    ])
