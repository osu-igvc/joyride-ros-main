# Python
from http.server import executable
import os
from ament_index_python.packages import get_package_share_directory

# ROS
from launch import LaunchDescription
from launch_ros.actions import Node
#from launch.action import DeclareLaunchArgument
#from launch.substitutions import TextSubstitution


def generate_launch_description():

    # example_front_video_path = '/home/igvcsp2022/Documents/igvc_main_software/testing_data/dashcam_3.mp4'

    # front_camera_src_arg = DeclareLaunchArgument(
    #     'front_camera_source', default_value=TextSubstitution(text='video0')
    # )
    # front_camera_topic_arg = DeclareLaunchArgument(
    #     'front_camera_topic', default_value=TextSubstitution(text='/cameras/front_raw')
    # )

    sensor_config = os.path.join(
        get_package_share_directory('joyride_core'),
        'config',
        'sensor_config.yaml'
    )

    return LaunchDescription([

        # ----------- Publish Raw Camera Streams ----------- #
        Node(
            package='joyride_sensors_generic',
            namespace='sensors/cameras',
            executable='raw_image_publisher',
            name='front_camera_pub_node',
            parameters=[sensor_config]
        ),

        # ----------- Publish Lidar Streams ----------- #

    ])