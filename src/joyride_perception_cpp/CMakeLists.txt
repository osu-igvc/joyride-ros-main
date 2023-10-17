cmake_minimum_required(VERSION 3.8)
project(joyride_perception_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include/joyride_perception_cpp)

add_executable(white_detection_node src/white_detection_node.cpp)
ament_target_dependencies(white_detection_node rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)
install(TARGETS
white_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(lane_slider_node src/lane_slider_node.cpp)
ament_target_dependencies(lane_slider_node rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)
install(TARGETS
  lane_slider_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(lane_roi_node src/lane_roi_set.cpp)
ament_target_dependencies(lane_roi_node rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)
install(TARGETS
lane_roi_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
