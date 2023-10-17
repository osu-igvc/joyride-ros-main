from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    lifecycle_nodes = [ '/sensors/cameras/bfly_center']
    autostart = True

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

        # Lifecycle
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_bfly',
            output='screen',
            parameters=[{'autostart': autostart}, {'node_names':lifecycle_nodes}, {'bond_timeout':0.0}]
        )
    ])