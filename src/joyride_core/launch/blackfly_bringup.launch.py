# Python
from http.server import executable
import os
from ament_index_python.packages import get_package_share_directory

# ROS
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
#from launch.action import DeclareLaunchArgument
#from launch.substitutions import TextSubstitution


def generate_launch_description():

    blackfly_config = os.path.join(
        get_package_share_directory('joyride_core'),
        'config',
        'blackfly_cam_config.yaml'
    )

    return LaunchDescription([

        # ----------- Publish Raw Blackfly Camera Streams ----------- #
        # Node(
        #     package='joyride_blackfly_gige',
        #     namespace='sensors/cameras',
        #     executable='blackfly_gige',
        #     name='lane_camera_img_pub',
        #     parameters=[blackfly_config]
        # ),

        # Node(
        #     package='joyride_blackfly_gige',
        #     namespace='sensors/cameras',
        #     executable='blackfly_gige',
        #     name='left_sign_camera_img_pub',
        #     parameters=[blackfly_config]
        # ),

        Node(
            package='joyride_blackfly_gige',
            namespace='sensors/cameras',
            executable='blackfly_gige',
            name='right_sign_camera_img_pub',
            parameters=[blackfly_config]
        ),

        # ----------- Blackfly Static Transforms ----------- #


        
        # EVENTUALLY PUT IN URDF


        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_link_tf_publisher',
        #     arguments=['0', '0.1016', '0.127', '0', '0', '0', 'base_link', 'camera_link']
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='vectornav_tf_publisher',
        #     arguments=['-0.4826', '-0.1651', '0.1143', '0', '0', '0', 'base_link', 'vectornav']
        # ),
    ])