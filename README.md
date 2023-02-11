# Joyride ROS Main

This repository is the primary software entry point for Oklahoma State University's first entry into the Intelligent Ground Vehicle Competition (IGVC). The software stack for Joyride is based on ROS2 - Foxy. It utilizes Navigation2 as its primary navigation stack, with custom packages to implement other functionality.

**This is a work-in-progress autonomy stack, missing significant functionality**

The repository is organized in the following manner:

**joyride-ros-main**
- joyride_interfaces
- joyride_servers
- joyride_sensors
- joyride_perception
- joyride_navigation
- joyride_hmi
- joyride_util


## Joyride_Interfaces

Holds ROS message and action definitions. Used to maintain consistency accross packages.

## Joyride_Servers

Holds "servers" that manage system-wide operation. For example, the *usb-link-node* is here, which converts and transmits messages over USB to the drive by wire system.

## Joyride_Sensors

Sensor driver nodes. LIDAR, GPS, cameras, and more. Launch files to bringup the sensors with proper transforms.

### Vectornav VN300 GPS/IMU

Can provide IMU data at up to 400Hz. Requires some careful configuration within the vectornav node's YAML. Namely, minimize data being transferred.
- rateDivisor = 1 (send at frequency of 400Hz/1Hz = 400Hz)
- commonField = 0x0ff0 (only send specific data types)
- asyncMode = 1 (only send to serial 1)

## Joyride_Perception

Perception systems - computer vision, SLAM nodes, estimation, etc.

## Joyride_Navigation

Navigation systems - planners, etc.

## Joyride_HMI

Human-machine interface nodes. Joystick control and the onboard graphical user interface (GUI).