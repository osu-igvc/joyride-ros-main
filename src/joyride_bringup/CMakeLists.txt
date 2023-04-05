cmake_minimum_required(VERSION 3.5)
project(joyride_bringup)

find_package(ament_cmake REQUIRED)
#find_package(joyride_common REQUIRED)
#find_package(joyride REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
