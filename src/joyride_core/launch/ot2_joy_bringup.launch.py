
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # -------------------- Common -------------------- #

    roscan_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('joyride_core'), 'launch'),
        '/can_server_bringup.launch.py'
        ])
    )

    rosbagger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('joyride_core'), 'launch'),
        '/data_log_bringup.launch.py'
        ])
    )

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('joyride_core'), 'launch'),
        '/static_transforms.launch.py'
        ])
    ),

    # -------------------- Sensors -------------------- #

    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('vectornav'), 'launch'),
        '/vectornav.launch.py'
        ])
    )

    blackfly_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('joyride_core'), 'launch'),
        '/blackfly_bringup.launch.py'
        ])
    )

    # -------------------- Localization -------------------- #

    localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('joyride_core'), 'launch'),
        '/joyride_localization.launch.py'
    ])),


    # -------------------- Joystick -------------------- #
    
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('joyride_hmi'), 'launch'),
        '/joystick_bringup.launch.py'
        ])
    )

    return LaunchDescription([
        
        Node(
            package='joyride_control',
            executable='vel_preprocess_node',
            name='vel_node',
        ),

        static_tf_launch,
        rosbagger_launch,
        roscan_server_launch,
        
        vectornav_launch,
        blackfly_launch,

        localizer_launch,

        joy_launch
    ])
