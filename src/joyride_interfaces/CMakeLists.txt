cmake_minimum_required(VERSION 3.8)
project(joyride_interfaces)

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
  "msg/DriveByWireSystemInfo.msg"
  "msg/VelocityState.msg"

  # Health/diagnostic Messages
  "msg/SystemDiagnosticSummary.msg"
  "srv/RequestAutoEnableDisable.srv"
  DEPENDENCIES std_msgs
)

# --------------------------------------------------------- #

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
