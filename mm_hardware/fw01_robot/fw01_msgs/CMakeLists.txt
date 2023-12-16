cmake_minimum_required(VERSION 3.8)
project(fw01_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package(action_msgs REQUIRED)

set(msg_files
  msg/BatteryStates.msg
  msg/RobotStates.msg
  msg/IOStates.msg
  msg/SteeringCtrlFb.msg
  msg/Freedrive.msg
  msg/FreedriveJoint.msg
  #msg/FreedriveCmd.msg
)

set(srv_files
  srv/IOCmd.srv
  srv/SteerCmd.srv
  srv/TransverseCmd.srv
  srv/SteeringCtrlCmd.srv
  srv/FreedriveCmd.srv
  srv/FreedriveIndividualCmd.srv
)

set(action_files
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    #action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
