# Created: Spring 2023, comments added 12/13/23 by Josephine Wade
# This file creates the Nav2 static transforms; it is called to publish the TF when using Nav2
# Requires:
#   - Configured YAML file for Nav2 static transforms (will need to be updated when sensors are added)

# Python imports
from http.server import executable
import os
from ament_index_python.packages import get_package_share_directory

# ROS imports
import launch
import launch_ros.actions

def generate_launch_description():

    transform_config = os.path.join(
        get_package_share_directory('joyride_servers'),
        'config',
        'transforms.yaml'
    )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='joyride_servers',
            output='screen',
            namespace='servers',
            executable='joyride_static_tf_broadcaster',
            name='static_transform_node',
            parameters=[transform_config]),
    ]
  )