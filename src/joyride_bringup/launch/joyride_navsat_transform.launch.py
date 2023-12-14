# Created: Spring 2023, comments added 12/13/23 by Josephine Wade
# Unclear if this file is needed anymore since GPS localization file provides the Nav transforms and localization
# Provides:
#   - Nav2 transforms data to GPS coordinates and world frame
# Requires:
#   - nav localization configuration file
# SUBSCRIBED TOPICS:
#   - /imu
#   - /gps/fix
#   - /odometry/gps
# PUBLISHED TOPICS
#   - /gps/filtered
#   - /odometry/filtered
#   - /odom      (published by navsat_odom)
#   - /local_enu (published by navsat_odom)

from launch import LaunchDescription
import launch_ros.actions
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    localization_params = os.path.join(get_package_share_directory('joyride_localization'), 'config', 'nav_localization_config.yaml')

    return LaunchDescription([

        # Nav Transforms
        launch_ros.actions.Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
            output='screen',
            parameters=[localization_params],
            remappings=[('imu', 'vectornav/imu'),# Input. From sensor.
                        ('gps/fix', 'vectornav/gnss'),      # Input. From sensor.
                        ('gps/filtered', 'gps/filtered'),   # Output (optional). Convert to GPS coords.
                        ('odometry/gps', 'odometry/gps'),   # Output. coords transformed into world frame.
                        ('odometry/filtered', 'odom')]      # Input. From odom EKF
        ),
])