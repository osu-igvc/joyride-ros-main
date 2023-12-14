# Created: Spring 2023, comments added 12/13/23 by Josephine Wade
# This file creates and launches nodes for the Drive-By-Wire system 
# This file should be launched when the computer starts up (linux autostart program)
# If the GUI does not show feedback (steering angle is easiest to check by turning the wheel)
# Then relaunch the GUI application (the app labeled "joyride gui" with a settings wheel icon)
# If it still does not connect then relaunch the CAN connection by launching this file in a terminal; it will have to remain open to keep the connection

# Python imports
from http.server import executable
import os
from ament_index_python.packages import get_package_share_directory

# ROS imports
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch_ros.actions import Node, LifecycleNode
from launch.events import matches_action

from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessStart
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    can_config = os.path.join(
        get_package_share_directory('joyride_ros2_socketcan'),
        'config',
        'can_msg_config.yaml'
    )

    return LaunchDescription([

        # ----------- ROSCAN Server ----------- #

        Node(
            package='joyride_ros2_socketcan',
            output='screen',
            namespace='servers',
            executable='roscan_server',
            name='roscan_interface',
            parameters=[can_config],
        ),

    ])