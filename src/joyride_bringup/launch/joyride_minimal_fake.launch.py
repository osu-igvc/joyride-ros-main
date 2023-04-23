# A minimal launch file for the Joyride project
# Includes the following nodes:
# - CAN Server
# - Transforms
# - Diagnostics (aggregator, automode manager, computer monitor)
# - Vectornav INS
# - GPS-based Localization
# - Navsat Transform

# The intention is that all of these should be launched regardless of intended use.
# This is for outdoor usage, so the GPS-based localization is included.
# For indoor usage, the minimal_indoor launch file should be used instead, which uses our fake_odometry node instead of the GPS-based localization.


# Python
import os
from ament_index_python.packages import get_package_share_directory

# ROS
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

        # Static transforms
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_core'), 'launch'),
            '/static_transforms.launch.py'
            ])
        ),

        # CAN Server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/CAN.launch.py'
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

        # Vectornav INS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('vectornav'), 'launch'),
            '/vectornav.launch.py'
            ])
        ),

        # Faked Localization
        Node(
            package='joyride_localization',
            executable='fake_odom',
            name='fake_odom',
            output='screen',
        )
])