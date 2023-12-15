# Created: Spring 2023, comments added 12/13/23 by Josephine Wade
# A minimal launch file for the Joyride project
# Includes the following nodes:
# - Transforms
# - Diagnostics (aggregator, automode manager, computer monitor)
# - Vectornav INS
# - GPS-based Localization
# - Navsat Transform

# The intention is that all of these should be launched regardless of intended use.
# This is for indoor usage, so the GPS-based localization is not inclued.
# For outdoor usage, the minimal launch file should be used instead, which uses GPS-based localization.


# Python imports
import os
from ament_index_python.packages import get_package_share_directory

# ROS imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    default_diagnostic = os.path.join(get_package_share_directory('joyride_bringup'), 'config', 'diagnostic_minimal.yaml')

    diagnostic_config = LaunchConfiguration('diagnostic_config', default=default_diagnostic)

    diagnostic_param_cmd = DeclareLaunchArgument(
        'diagnostic_config',
        default_value=default_diagnostic,
        description='Path to diagnostic config file'
    )


    return LaunchDescription([
        diagnostic_param_cmd,


         # ROSBAGGER
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #          get_package_share_directory('joyride_bringup'), 'launch'),
        #          '/rosbag_recorder.launch.py']),
        #      launch_arguments={'bag_name':'F23_Nav2'}.items()
        # ),

        # Static transforms
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/joyride_transforms.launch.py'
            ])
        ),


        # Diagnostics
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/joyride_diagnostics.launch.py'
            ]),
            launch_arguments={'diagnostic_config': diagnostic_config}.items()
        ),

         # Sensors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('joyride_bringup'), 'launch'),
                '/joyride_sensors.launch.py'])
        ),

        # Faked Localization
        Node(
            package='joyride_localization',
            executable='fake_odom',
            name='fake_odom',
            output='screen',
        ),
        
        # Velocity preprocessor
        Node(
            package='joyride_control_py',
            executable='vel_preprocessor',
            name='vel_node',
        ),
])