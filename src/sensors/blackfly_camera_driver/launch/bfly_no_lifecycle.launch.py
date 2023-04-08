from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    camera_params = os.path.join(get_package_share_directory('blackfly_camera_driver'), 'config', 'bfly_lifecycle_config.yaml')

    return LaunchDescription([
        LifecycleNode(
            package='blackfly_camera_driver',
            executable='blackfly_camera_driver',
            name='bfly_center',
            namespace='sensors/cameras',
            output='screen',
            parameters=[camera_params]
        ),

        LifecycleNode(
            package='blackfly_camera_driver',
            executable='blackfly_camera_driver',
            name='bfly_left',
            namespace='sensors/cameras',
            output='screen',
            parameters=[camera_params]
        ),

        LifecycleNode(
            package='blackfly_camera_driver',
            executable='blackfly_camera_driver',
            name='bfly_right',
            namespace='sensors/cameras',
            output='screen',
            parameters=[camera_params]
        ),

    ])