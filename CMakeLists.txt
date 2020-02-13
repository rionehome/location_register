cmake_minimum_required(VERSION 3.5)

project(location_register)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-O3 -Wall -Wextra -Wpedantic -Wunused-variable -Wunused-parameter -Wpedantic -Wdeprecated-declarations)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rione_msgs REQUIRED)

include_directories(include)

add_library(location_register SHARED src/location_register.cpp)
ament_target_dependencies(location_register rclcpp tf2_ros rione_msgs)

add_executable(location_register_node src/location_register_node.cpp src/location_register.cpp)
ament_target_dependencies(location_register_node rclcpp tf2_ros std_msgs geometry_msgs nav_msgs sensor_msgs rione_msgs)

target_include_directories(location_register_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS location_register_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
