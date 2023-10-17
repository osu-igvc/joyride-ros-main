

import launch
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():

    aggregator_params = os.path.join(get_package_share_directory('joyride_servers'), 'config', 'diagnostics_standard.yaml')
    manager_params = os.path.join(get_package_share_directory('joyride_servers'), 'config', 'automode_manager_standard.yaml')

    diagnostic_agg = Node(
        package='diagnostic_aggregator',
        name='diagnostic_analyzers',
        executable='aggregator_node',
        output='screen',
        parameters=[aggregator_params]
    )

    automode_manager = Node(
        name='automode_manager_server',
        package='joyride_servers',
        executable='automode_manager',
        parameters=[manager_params]
    )


    return launch.LaunchDescription([
        diagnostic_agg,
        automode_manager


    ])
