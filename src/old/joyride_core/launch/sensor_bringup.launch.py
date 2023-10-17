# Python
from http.server import executable
import os
from ament_index_python.packages import get_package_share_directory

# ROS
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch_ros.actions import Node, LifecycleNode
from launch.events import matches_action

from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessStart
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    vectornav_config = os.path.join(
        get_package_share_directory('joyride_core'),
        'config',
        'vectornav_gps_imu.yaml'
    )

    hokuyo_urg_lifecycle = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name='front_lidar',
        namespace='sensors/lidar',
        remappings=[('scan', 'scan')], #don't remap right now
        parameters=[sensor_config],
        output='screen',
    )

    urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=hokuyo_urg_lifecycle,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hokuyo_urg_lifecycle),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=hokuyo_urg_lifecycle,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hokuyo_urg_lifecycle),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_start')),
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py'
        ]),
        launch_arguments={
            'depth_align.enable': 'True',
            'depth_module.profile': '640x480x30',
            'rgb_module.profile': '640x480x30',
            'enable_accel': 'True',
            'enable_gyro': 'True',
            'depth_module.enable_auto_exposure': 'True',
            'linear_accel_cov': '1.0',
            'unite_imu_method': '2' #0: none, 1: copy, 2: linear_interpolation
        }.items()
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

        # # ----------- Vectornav GPS/IMU ----------- #
        Node(
            package='vectornav', executable='vectornav',
            output='screen',
            parameters=[vectornav_config]
        ),

        Node(
            package='vectornav', executable='vn_sensor_msgs',
            output='screen',
            parameters=[vectornav_config]
        ),

        # ----------- Realsense Depth Camera ----------- #
        DeclareLaunchArgument('depth_module.enable_auto_exposure', default_value='true'),
        realsense_launch,

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_imu_tf_publisher',
            arguments=['0', '0', '0', '-1.57079', '0', '1.57079', 'camera_link', 'camera_imu_optical_frame']
        ),

        # ----------- Publish Lidar Streams ----------- #
        DeclareLaunchArgument('auto_start', default_value='true'),
        hokuyo_urg_lifecycle,
        urg_node2_node_activate_event_handler,
        urg_node2_node_configure_event_handler,

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_front_tf_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'hokuyo_front']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_tf_publisher',
            arguments=['0', '0.1016', '0.127', '0', '0', '0', 'base_link', 'camera_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vectornav_tf_publisher',
            arguments=['-0.4826', '-0.1651', '0.1143', '0', '0', '0', 'base_link', 'vectornav']
        ),
    ])