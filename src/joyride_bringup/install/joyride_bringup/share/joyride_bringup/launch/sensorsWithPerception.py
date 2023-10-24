from launch import LaunchDescription            
from launch_ros.actions import Node                       
from launch.actions import IncludeLaunchDescription                           
import os
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
       


       #DO WE NEED TO LAUNCH THIS?
       # Minimal
        #IncludeLaunchDescription(
          #  PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('joyride_bringup'), 'launch'),'/joyride_minimal.launch.py'])
        #),



        # Lidar 2D
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('joyride_bringup'), 'launch'),'/lidar2D.launch.py'])
        ),


        #Cameras
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('blackfly_camera_driver'), 'launch'),'/bfly_center_lifecycle.launch.py'])
        ),


        #Lane detection slider 
        Node(
            package='joyride_perception_cpp',
            executable='lane_slider_node',
            namespace='perception',
            name='slider'
        ),


        #Run Lane Detection Node
        Node(
            package='joyride_perception_cpp',  #package our node is in
            executable='white_detection_node', #node we want to run
            namespace='perception',
            name='White_Detection'
        ),
        #Syntax for bringing a node up in a launch file 

        #Blob Detection Node 
        Node(
            package='joyride_perception', #packge that blob detector is in
            executable='blob_detector',   #Node we want to run 
            namespace='perception',       
            name='blob_detector'
        )
        





    ])
