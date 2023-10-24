

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

    controller_type = LaunchConfiguration('controller_type')
    nav_params = LaunchConfiguration('nav2_params')

    declare_controller_type_cmd = DeclareLaunchArgument(
        'controller_type',
        default_value='purepursuit_config.yaml'
    )

    final_nav_params_path = DeclareLaunchArgument(
        'nav2_params',
        default_value=[get_package_share_directory('joyride_bringup'), '/config/',LaunchConfiguration('controller_type')])

    return LaunchDescription([
        declare_controller_type_cmd,
        final_nav_params_path,

        # CAN
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/CAN.launch.py'
            ])
        ),

        # # Transforms - Possibly Old version; throws TF error
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #     get_package_share_directory('joyride_core'), 'launch'),
        #     '/static_transforms.launch.py'
        #     ])
        # ),

        # Transforms - Static TFs
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #     get_package_share_directory('joyride_bringup'), 'launch'),
        #     '/joyride_transforms.launch.py'
        #     ])
        # ), 

        # GPS/IMU
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #     get_package_share_directory('vectornav'), 'launch'),
        #     '/vectornav.launch.py'
        # ])
        # ),

        # Control
        Node(
           package='joyride_control_py',
           executable='vel_preprocessor',
           name='vel_node',
        ),
        

        # Localization
        IncludeLaunchDescription(
           PythonLaunchDescriptionSource([os.path.join(
           get_package_share_directory('joyride_bringup'), 'launch'),
           '/gps_localization.launch.py']),
        ),
        
        # Faked Localization
        #Node(
        #     package='joyride_localization',
        #     executable='fake_odom',
        #     name='fake_odom',
        #     output='screen',
        #),

        # Navstack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_bringup'), 'launch'),
            '/navstack.launch.py']),
            launch_arguments={'params_file':nav_params}.items()
        ),
        # # Cameras
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([os.path.join(
        #        get_package_share_directory('blackfly_camera_driver'), 'launch'),
        #        '/bfly_center_lifecycle.launch.py'
        #    ])
        #    ),

        # 2D Lidar
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([os.path.join(
        #        get_package_share_directory('joyride_bringup'), 'launch'),
        #        '/lidar2D.launch.py'
        #   ])
        #),
    ])
