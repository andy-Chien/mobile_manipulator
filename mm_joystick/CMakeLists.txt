cmake_minimum_required(VERSION 3.5)
project(mm_joystick)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mm_msgs REQUIRED)
find_package(rclcpp REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  sensor_msgs
  mm_msgs
  rclcpp
)

add_library(${PROJECT_NAME} SHARED
  src/mm_joystick.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(
  ${PROJECT_NAME}_node
  src/mm_joystick_node.cpp
)

target_include_directories(
  ${PROJECT_NAME}_node
  PRIVATE
  include
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY config launch include
	DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()