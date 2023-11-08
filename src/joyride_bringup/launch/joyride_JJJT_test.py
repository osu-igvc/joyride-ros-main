# A test launch file for the Joyride project Fall 2023 
# Includes the following nodes:
# - CAN Server
# - Diagnostics (aggregator, automode manager, computer monitor)
# - Transforms
# - ROS Bagger
# - Cameras
# - Lidar 2D
# - Vectornav INS
# - Faked GPS
# - GPS-based Localization
# - Vel Preprocessor
# - Static Transform
# - Joystick 
# - Preception
# - Navsat Transform
# - Simple Pose Controller 
# - Navstack
 

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