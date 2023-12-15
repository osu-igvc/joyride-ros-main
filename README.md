# Joyride ROS Main

This repository is the primary software entry point for Oklahoma State University's first entry into the Intelligent Ground Vehicle Competition (IGVC). The software stack for Joyride is based on ROS2 - Foxy. It utilizes Navigation2 as its primary navigation stack, with custom packages to implement other functionality.

**This is a work-in-progress autonomy stack, missing significant functionality**

The repository is organized in the following manner:

**joyride-ros-main/src**
- joyride
- joyride_bringup
- joyride_control [TODO]
- joyride_interfaces
- joyride_hmi
- joyride_interfaces
- joyride_localization
- joyride_perception
- joyride_ros2_socketscan [TODO]
- joyride_servers
- joyride_test [TODO]
- joyride_vectornav
- mbtiles holder
- old
- sensors
- third_party

## joyride
A metapackage describing the overall system's dependencies.

## joyride_bringup
A package that contains launch files used for launching various different funtions of the car

## joyride_control
TODO

## Joyride_Interfaces
Holds ROS message and action definitions. Used to maintain consistency across packages.

## Joyride_HMI
Human-machine interface nodes. Joystick control and the onboard graphical user interface (GUI).

## joyride_localization
Holds ros message publisher for gps localization and transform.

## Joyride_perception
Perception systems - computer vision, SLAM nodes, estimation, etc.

## joyride_ros2_socketscan
TODO

## Joyride_servers
Holds "servers" that manage system-wide operation. For example, the *usb-link-node* is here, which converts and transmits messages over USB to the drive by wire system.

## joyride_tests
TODO

## joyride_vectornav
#### Vectornav VN300 GPS/IMU
Can provide IMU data at up to 400Hz. Requires some careful configuration within the vectornav node's YAML. Namely, minimize data being transferred.
- rateDivisor = 1 (send at frequency of 400Hz/1Hz = 400Hz)
- commonField = 0x0ff0 (only send specific data types)
- asyncMode = 1 (only send to serial 1)

## old
Contains code from before the workspace was converted to ROS 2 humble

## mbtiles
Containes gps maps of given areas. tracked through git lfs as gps files exceed github file size limit of 100 Mb

## third_party
Sensor driver nodes. LIDAR, GPS, cameras, and more form third party sources.





## Example Launch

To launch minimal functionality of joyride for outdoors
```bash
  ros2 launch joyride_bringup joyride_minimal.launch.py
```

To launch minimal functionality of joyride for indoors
```bash
  ros2 launch joyride_bringup joyride_minimal_fake.launch.py
```

For visual of cameras and other ros data
```bash
  rviz
```

For ROS2 message debugging and message filtering
```bash
  ros2 run rqt_console rqt_console
```
