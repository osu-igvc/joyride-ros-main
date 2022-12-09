from http.server import executable
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    #util_dir = get_package_share_directory('joyride_hmi')


    return LaunchDescription([

        # ----------- Publish Joystick Info ----------- #
        Node(
            package='joyride_hmi',
            executable='joystick_pub_node',
            name='Joystick_Publisher'
        ),

        # ----------- Publish Joystick Info ----------- #
        Node(
            package='joyride_hmi',
            executable='joystick_mapper_node',
            name='Joystick_Velocity_Mapper'
        ),

    ])