cmake_minimum_required(VERSION 3.5)
project(joyride_interfaces)

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
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)





# ---------------- Add new interfaces here ---------------- #

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Heartbeat.msg"
  "action/RequestEnableDisable.action"
  "action/RequestAudioVisual.action"
  "msg/Health.msg"
  "msg/TorqueEPSFeedback.msg"
  "msg/AccessoriesGEMFeedback.msg"
  "msg/EPSPositionVelocityFeedback.msg"

  # Health/diagnostic Messages
  "msg/SystemDiagnosticSummary.msg"
  "srv/RequestAutoEnableDisable.srv"
  DEPENDENCIES std_msgs
)

# --------------------------------------------------------- #


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
