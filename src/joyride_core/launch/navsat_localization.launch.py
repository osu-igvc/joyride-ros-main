from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_localization_dir = get_package_share_directory('joyride_core')
    parameters_file_dir = os.path.join(robot_localization_dir, 'config')
    parameters_file_path = os.path.join(parameters_file_dir, 'nav_localization_config.yaml')
    os.environ['FILE_PATH'] = str(parameters_file_dir)
    
    static_tf_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('joyride_core'), 'launch'),
            '/static_transforms.launch.py'
        ])
    )
    
    
    return LaunchDescription([
    launch.actions.DeclareLaunchArgument(
        'output_final_position',
        default_value='false'),
    launch.actions.DeclareLaunchArgument(
        'output_location',
    default_value='~/dual_ekf_navsat_example_debug.txt'),

    static_tf_launch,

    launch_ros.actions.Node(
        package='joyride_core',
        executable='clock_bag_repub',
        name='clock_bag_node',
        output='screen'
    ),

    launch_ros.actions.Node(
        package='joyride_odometry',
        executable='navsat_odom',
        name='navsat_odom_node',
        output='screen'
    ),

    launch_ros.actions.Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('imu', 'vectornav/imu'),              # Input. From sensor.
                        ('gps/fix', 'vectornav/gnss'),              # Input. From sensor.
                        ('gps/filtered', 'gps/filtered'),         # Output (optional). Convert to GPS coords.
                        ('odometry/gps', 'odometry/gps'),           # Output. coords transformed into world frame.
                        ('odometry/filtered', 'odom')]    # Input. From odom EKF
           ),

])