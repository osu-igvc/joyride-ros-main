from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(package='blackfly_camera_driver', executable='blackfly_camera_driver',
                      name='bfly_node', namespace='', output='screen')
    ])