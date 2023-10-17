

import launch
import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():

    comp_params = os.path.join(get_package_share_directory('joyride_servers'), 'config', 'computer_monitor_standard.yaml')

    return launch.LaunchDescription([
        Node(
            name='computer_monitor_server',
            package='joyride_servers',
            executable='computer_monitor',
            parameters=[comp_params]
        )
    ])
