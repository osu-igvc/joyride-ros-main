# Created: Spring 2023, comments added 12/13/23 by Josephine Wade
# A launch file for testing out the Model Predictive Path Integral (MPPI) controller for Nav2
# Launches:
#   - Minimal (static Tfs, Diagnostics, sensors, localization, and velocity preprocessor)
#   - NavStack 
# Requires:
#   - Configured parameter file for MPPI controller

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

    controller_type = LaunchConfiguration('controller_type')
    nav_params = LaunchConfiguration('nav_params')

    declare_controller_type_cmd = DeclareLaunchArgument(
        'controller_type',
        default_value='mppi_config.yaml'
    )

    final_nav_params_path = DeclareLaunchArgument(
        'nav_params',
        default_value=[get_package_share_directory('joyride_bringup'), '/config/',LaunchConfiguration('controller_type')])

    return LaunchDescription([
        declare_controller_type_cmd,
        final_nav_params_path,

        # Minimal
        IncludeLaunchDescription(
            
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/joyride_minimal.launch.py'
            ])
        ),
        # Navstack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/joyride_navstack.launch.py']),
            launch_arguments={'params_file':nav_params}.items()
        )
    ])
