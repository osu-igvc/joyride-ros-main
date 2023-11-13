#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/joyride-obc/joyride-ros-main/install/setup.bash

ros2 launch joyride_bringup CAN.launch.py

