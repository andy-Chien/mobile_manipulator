cmake_minimum_required(VERSION 3.8)
project(fw01_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(fw01_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  rclcpp_lifecycle
  lifecycle_msgs
  fw01_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  rclcpp
)

add_library(${PROJECT_NAME} SHARED
  src/freedrive_controller.cpp
  src/steer_controller.cpp
  src/gpio_controller.cpp)

add_executable(move_joy_node
    test/move_joy.cpp
  )
  
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

target_include_directories(move_joy_node PRIVATE
  include
)

target_link_libraries(move_joy_node ${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_target_dependencies(move_joy_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#rosidl_target_interfaces(${PROJECT_NAME} "rosidl_typesupport_cpp")

# prevent pluginlib from using boost
#target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS move_joy_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
