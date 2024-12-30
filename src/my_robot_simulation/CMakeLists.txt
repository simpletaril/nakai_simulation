cmake_minimum_required(VERSION 3.8)
project(my_robot_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament CMake and Gazebo dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)  # Required for Gazebo plugins
find_package(gazebo_plugins REQUIRED)  # Required for Gazebo plugins
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# Add executables
add_executable(force_applier src/force_applier.cpp)
add_executable(bug2 src/bug2.cpp)
add_executable(shape_detection_node src/shape_detection_node.cpp)

# Specify dependencies for each executable separately
ament_target_dependencies(force_applier
  rclcpp
  geometry_msgs
  std_msgs
  gazebo_ros
  sensor_msgs
  gazebo_plugins
)

ament_target_dependencies(bug2
  rclcpp
  geometry_msgs
  std_msgs
  gazebo_ros
  sensor_msgs
  gazebo_plugins
)


ament_target_dependencies(shape_detection_node 
  rclcpp 
  sensor_msgs 
  OpenCV 
  cv_bridge)

# Install targets
install(TARGETS
  force_applier
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  bug2
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS
  shape_detection_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
