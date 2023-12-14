# Created: Fall 2023, comments added 12/13/23 by Josephine Wade

# This was a test file to try to condense the minimal and minimal_fake file by having a launch argument to decide which localization to use
# Not finished - once it is done this method could be used to condense a lot of launch files 

# A minimal launch file for the Joyride project
# Includes the following nodes:
# - CAN Server
# - Transforms
# - Diagnostics (aggregator, automode manager, computer monitor)
# - Vectornav INS
# - GPS-based Localization
# - Navsat Transform

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
# For indoor usage, the minimal_fake launch file should be used instead, which uses our fake_odometry node instead of the GPS-based localization.


# Python
import os
from ament_index_python.packages import get_package_share_directory

# ROS
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Add DeclareLaunchArguments to the LaunchDescription
# LaunchDescription(Nodes, LaunchArguments) or something similar
FAKE_LOCOLIZATION = True


def generate_launch_description():

    default_diagnostic = os.path.join(get_package_share_directory('joyride_bringup'), 'config', 'diagnostic_minimal.yaml')

    diagnostic_config = LaunchConfiguration('diagnostic_config', default=default_diagnostic)

    diagnostic_param_cmd = DeclareLaunchArgument(
        'diagnostic_config',
        default_value=default_diagnostic,
        description='Path to diagnostic config file'
    )

    Nodes = []
    LaunchArguments = []

    #Include launch arguments
    LaunchArguments.append(diagnostic_param_cmd)

    # Include other launch files
    Nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('joyride_bringup'), 'launch'),'/joyride_transforms.launch.py'])))

    Nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('joyride_bringup'), 'launch'),
                '/joyride_diagnostics.launch.py']),
            launch_arguments={'diagnostic_config': diagnostic_config}.items()
        ))
    
    Nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('joyride_bringup'), 'launch'),
                '/joyride_sensors.launch.py'])
        ))

    # Add independant nodes
    Nodes.append(Node(
            package='joyride_control_py',
            executable='vel_preprocessor',
            name='vel_node',
        ))


    # Switch between fake and gps localization
    if FAKE_LOCOLIZATION:
        Nodes.append(Node(
            package='joyride_localization',
            executable='fake_odom',
            name='fake_odom',
            output='screen',
        ))
    
    else:
        Nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('joyride_bringup'), 'launch'),
                '/gps_localization.launch.py'])
        ))

    Launch = Nodes

    # Return all required nodes for basic operation
    return LaunchDescription(Launch)