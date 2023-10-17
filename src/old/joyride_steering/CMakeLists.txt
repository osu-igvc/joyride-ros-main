cmake_minimum_required(VERSION 3.5)
project(joyride_steering)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(joyride_control_lib REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

###########################################################################################
include_directories(include)
# define the binary to be built and identify the source files with with which to build it
add_executable(joyride_steering src/steering_pid.cpp)
# tell CMake that the executable "main" depends on the library "my_value_converter_library"
ament_target_dependencies(joyride_steering joyride_control_lib rclcpp std_msgs)

# install the executable in the lib folder to make it detectable through setup.bash
install(TARGETS 
joyride_steering
  DESTINATION lib/${PROJECT_NAME}/
)
###########################################################################################


ament_package()
