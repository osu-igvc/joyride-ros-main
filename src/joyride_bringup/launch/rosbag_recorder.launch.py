# Created: Spring 2023, comments added 12/14/23 by Josephine Wade
# This file records rosbags in a folder with a .db3 and YAML file that can be played back
# Another way of recording bags is to use "ros2 bag record -s mcap --all" or another variant
# MCAP bags are able to be played back through Foxglove quicker than .db3 files
# LAUNCH CONFIG:
#   - "final_bag_name" can be specified when called to name the bag whatever you want
import launch
from datetime import datetime

from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    time = datetime.now()
    date_str = time.strftime("_%Y_%m_%d-%H_%M")
    bag_name = LaunchConfiguration('bag_name')
    final_bag_name = LaunchConfiguration('final_bag_name')

    declare_bagname_cmd = DeclareLaunchArgument(
        'bag_name',
        default_value='rosbag',
        description='Name of output bag file'
    )

    final_bag_file_cmd = DeclareLaunchArgument(
        'final_bag_name',
        default_value=[LaunchConfiguration('bag_name'), date_str])

    return launch.LaunchDescription([

        declare_bagname_cmd,
        final_bag_file_cmd,

        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '--include-hidden-topics', '-o', final_bag_name],
            output='screen'
        )
    ])