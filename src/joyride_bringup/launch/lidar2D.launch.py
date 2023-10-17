# Python
import os
from ament_index_python.packages import get_package_share_directory

# ROS
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch_ros.actions import LifecycleNode
from launch.events import matches_action

from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessStart
#from launch.action import DeclareLaunchArgument
#from launch.substitutions import TextSubstitution


def generate_launch_description():
    lidar_config = os.path.join(
        get_package_share_directory('joyride_bringup'),
        'config',
        'lidar_config.yaml'
    )

    hokuyo_urg_lifecycle = LifecycleNode(
            package='urg_node2',
            executable='urg_node2_node',
            name='front_lidar',
            namespace='sensors/lidar',
            remappings=[('scan', 'front_lidar/scan')],
            parameters=[lidar_config],
            output='screen',
        )

    urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=hokuyo_urg_lifecycle,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hokuyo_urg_lifecycle),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=hokuyo_urg_lifecycle,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hokuyo_urg_lifecycle),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('auto_start', default_value='true'),
        hokuyo_urg_lifecycle,
        urg_node2_node_activate_event_handler,
        urg_node2_node_configure_event_handler
    ])
