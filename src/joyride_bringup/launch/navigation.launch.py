
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

# Sets params file and controller type and its config
# Finds path of files needed
# Launches minimal - CAN, Diagnostics, Static Tfs, Sensors, Faked Localization for indoors
# Launches Navstack with params file. Possibly add in map file directory later. For now when launching this file you have to specify the map yaml path: 
# ros2 launch joyride_bringup navigation.launch.py map:=/home/joyride-obc/joyride-ros-main/src/joyride_bringup/maps/empty_map.yaml


def generate_launch_description():

    # controller_type = LaunchConfiguration('controller_type')
    nav_params = LaunchConfiguration('nav2_minimal_params')

    # declare_controller_type_cmd = DeclareLaunchArgument(
    #     'controller_type',
    #     default_value='purepursuit_config.yaml'
    # )

    final_nav_params_path = DeclareLaunchArgument(
        'nav2_minimal_params',
        default_value=[get_package_share_directory('joyride_bringup'), '/config/','nav2_minimal_params.yaml'])
    

    return LaunchDescription([
        # declare_controller_type_cmd,
        final_nav_params_path,

        # Minimal fake
        # IncludeLaunchDescription(
            
        #     PythonLaunchDescriptionSource([os.path.join(
        #     get_package_share_directory('joyride_bringup'), 'launch'),
        #     '/joyride_minimal_fake.launch.py'
        #     ])
        # ),
        # Minimal 
        IncludeLaunchDescription(
            
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/joyride_minimal.launch.py'
            ])
        ),

        # Navstack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/navstack.launch.py']),
            launch_arguments={'params_file':nav_params}.items()
        ),
        
    ])
