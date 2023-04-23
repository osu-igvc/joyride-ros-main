cmake_minimum_required(VERSION 3.8)
project(joyride_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(joyride_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

add_executable(velocity_preprocessor src/joyride_velocity_preprocessor_node.cpp src/joyride_velocity_preprocessor.cpp)

ament_target_dependencies(velocity_preprocessor
  rclcpp
  joyride_interfaces
  std_msgs
  geometry_msgs
  ackermann_msgs
  nav_msgs
)

target_include_directories(velocity_preprocessor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS 
  velocity_preprocessor
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

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
