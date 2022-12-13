# Python
from http.server import executable
import os
from ament_index_python.packages import get_package_share_directory

# ROS
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch.action import DeclareLaunchArgument
#from launch.substitutions import TextSubstitution


def generate_launch_description():

    vo_params=[{
          'frame_id':'base_link',
          'subscribe_depth':True,
          'publish_tf': False,
        #   'expected_update_rate': 30.0,
        #   'max_update_rate': 60.0,
        #   'min_update_rate': 15.0,
          'Vis/CorType': '1', #0: feature matching, 1: optical flow
          'Odom/Strategy': '1',
          'Odom/GuessMotion':'true',
          'Vis/EstimationType':'1',
          'use_sim_time': False,
          'approx_sync':False}]

    vo_remaps=[
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
          ('odom', '/sensors/vo_odom'),
          ]
    icp_params=[{
        'publish_tf':False,
        'use_sim_time':'False'
    }]
    icp_remaps=[
        ('scan', '/sensors/lidar/scan'),
        ('odom', 'sensors/icp_odom')
    ]

    


    ukf_config = os.path.join(
        get_package_share_directory('joyride_core'),
        'config',
        'ukf_config.yaml'
    )


    # ----------- Launch Cameras ----------- #
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_core'), 'launch'),
            '/core_full_sensor_bringup.launch.py'
        ])
    )

    slam_params = os.path.join(
        get_package_share_directory('joyride_core'),
        'config',
        'slam_toolbox_config.yaml'
    )




    return LaunchDescription([

        sensor_launch,

        # ----------- Visual odometry ----------- #
        Node(
            package='rtabmap_ros', executable='rgbd_odometry', output='screen',
            parameters=vo_params,
            remappings=vo_remaps
        ),

        # ----------- Laser odometry ----------- #
        # Node(
        #     package='rtabmap_ros', executable='icp_odometry', output='screen',
        #     remappings=icp_remaps,
        #     parameters=icp_params
        # ),



        
        # ----------- UKF ----------- #
        Node(
            package='robot_localization', executable='ukf_node', name='ukf_filter_node',
            output='screen',
            parameters=[ukf_config]
        ),


        # ----------- SLAM ----------- #

        Node(
            package='slam_toolbox', executable='sync_slam_toolbox_node',
            name='slam_toolbox_node',
            output='screen',
            parameters=[slam_params, {'use_sim_time': False}]
        )
    ])