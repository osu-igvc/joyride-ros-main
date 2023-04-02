

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

    params_path = os.path.join(get_package_share_directory('joyride_core'), 'config', 'nav_config.yaml')

    # -------------------- Common -------------------- #

        can = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_core'), 'launch'),
            '/can_server_bringup.launch.py'
        ])
        )

        comp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_servers'), 'launch'),
            '/computer_monitor.launch.py'
        ])
        )

        automode = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_core'), 'launch'),
            '/automode_manager.launch.py'
        ])
        )


    return LaunchDescription([

        can,
        comp,
        automode
    ])
