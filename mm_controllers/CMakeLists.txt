cmake_minimum_required(VERSION 3.5)
project(mm_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

add_compile_options(-g)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  controller_interface
  joint_trajectory_controller
  lifecycle_msgs
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  ur_dashboard_msgs
  ur_msgs
  tf2
  tf2_ros
  tf2_msgs
  mm_msgs
  moveit_msgs
  sensor_msgs
  Eigen3
  control_msgs
  control_toolbox
  generate_parameter_library
  hardware_interface
  rclcpp
  trajectory_msgs
  rclcpp_action
  rclcpp_components
  nav2_msgs
  nav_2d_utils
  nav_2d_msgs
  nav2_util
  nav2_core
  moveit_core
  moveit_ros_planning
  # robot_localization
)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(mm_controller_parameters
  src/mm_controller_parameters.yaml
  include/mm_controllers/validate_jtc_parameters.hpp
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/trajectory.cpp
  src/four_wheel_steering_kinematics.cpp
  src/mobile_path_controller.cpp
  src/mm_trajectory_controller.cpp
  src/mm_controller_utils.cpp
  src/mm_servo_controller.cpp
  src/mb_servo_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} 
  mm_controller_parameters
  # ${robot_localization_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(TARGETS ${PROJECT_NAME} mm_controller_parameters
  EXPORT export_mm_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  ${PROJECT_NAME}/base_init_pose_setter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(
  export_mm_controllers HAS_LIBRARY_TARGET
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
