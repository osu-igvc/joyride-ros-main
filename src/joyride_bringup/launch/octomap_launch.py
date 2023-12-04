import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[
                {'resolution': 0.05},
                {'frame_id': 'local_enu'},
                {'sensor_model/max_range': '15.0'}
            ],
            remappings=[
                ('cloud_in', '/perception/center/lanes_point_cloud')
            ]
        )
    ])

