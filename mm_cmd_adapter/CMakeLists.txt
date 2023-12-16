cmake_minimum_required(VERSION 3.8)
project(mm_cmd_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(mm_msgs REQUIRED)
find_package(mm_controllers REQUIRED)

add_executable(path_to_trajectory_server src/path_to_trajectory_server.cpp)

target_include_directories(path_to_trajectory_server PRIVATE
  ${mm_controllers_INCLUDE_DIRS}
)

ament_target_dependencies(path_to_trajectory_server
  rclcpp mm_msgs mm_controllers)

install(TARGETS
  path_to_trajectory_server
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/path_adapter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
