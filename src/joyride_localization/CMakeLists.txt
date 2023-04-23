cmake_minimum_required(VERSION 3.8)
project(joyride_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(joyride_interfaces REQUIRED)
find_package(vectornav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ackermann_msgs REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(ackermann_odom src/joyride_ackermann_odom.cpp)
add_executable(fake_odom src/joyride_fake_odom_node.cpp src/joyride_fake_odom.cpp)
add_executable(navsat_odom src/joyride_navsat_odom_node.cpp src/joyride_navsat_odom.cpp)


ament_target_dependencies(ackermann_odom 
  rclcpp 
  std_msgs 
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  nav_msgs
  joyride_interfaces
  vectornav_msgs
  tf2_geometry_msgs)

ament_target_dependencies(navsat_odom
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  geographic_msgs
  sensor_msgs
  tf2
  tf2_ros
  vectornav_msgs
  diagnostic_msgs
  diagnostic_updater
  tf2_geometry_msgs)

ament_target_dependencies(fake_odom
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  geographic_msgs
  sensor_msgs
  tf2
  tf2_ros
  vectornav_msgs
  diagnostic_msgs
  diagnostic_updater
  ackermann_msgs
  tf2_geometry_msgs)


target_link_libraries(navsat_odom
  ${Boost_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

target_include_directories(ackermann_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(navsat_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  target_include_directories(fake_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS 
  ackermann_odom
  navsat_odom
  fake_odom
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
