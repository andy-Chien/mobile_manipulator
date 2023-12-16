cmake_minimum_required(VERSION 3.8)
project(fw01_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
#find_package(mobile_base_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
## Declare a C++ library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/fw01_can.cpp
  src/fw01_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME} 
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#include_directories(${PROJECT_NAME}/include ${THIS_PACKAGE_INCLUDE_DEPENDS})
add_executable (fw01_can_test src/fw01_can_test.cpp src/fw01_can.cpp) 

ament_target_dependencies(
  fw01_can_test
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(
  fw01_can_test
  ${PROJECT_NAME} 
)

target_include_directories(
  fw01_can_test
  PRIVATE
  include
)




# Export hardware pligins
pluginlib_export_plugin_description_file(hardware_interface fw01_hardware_interface.xml)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS fw01_can_test
  DESTINATION lib/${PROJECT_NAME}
)


#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # comment the line when a copyright and license is added to all source files
#  set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # comment the line when this package is in a git repo and when
#  # a copyright and license is added to all source files
#  set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
