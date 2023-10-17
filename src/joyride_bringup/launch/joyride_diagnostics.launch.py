

import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

def generate_launch_description():

    default_params = os.path.join(get_package_share_directory('joyride_bringup'), 'config', 'diagnostic_minimal.yaml')

    diagnostic_config = LaunchConfiguration('diagnostic_config', default=default_params)

    declare_diag_config = DeclareLaunchArgument(
        'diagnostic_config',
        default_value=default_params
    )

    diagnostic_agg = Node(
        package='diagnostic_aggregator',
        name='diagnostic_analyzers',
        executable='aggregator_node',
        output='screen',
        parameters=[diagnostic_config]
    )

    automode_manager = Node(
        name='automode_manager_server',
        package='joyride_servers',
        executable='automode_manager',
        parameters=[diagnostic_config]
    )

    comp_monitor = Node(
        name='computer_monitor_server',
        package='joyride_servers',
        executable='computer_monitor',
        parameters=[diagnostic_config]
    )


    return launch.LaunchDescription([
        declare_diag_config,
        diagnostic_agg,
        automode_manager,
        comp_monitor
    ])
