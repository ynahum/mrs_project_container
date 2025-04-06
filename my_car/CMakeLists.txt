cmake_minimum_required(VERSION 3.5)
project(my_car)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  rclcpp_components
  rclpy 
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  nav_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Create Cpp executables

# Add executable for velocity control node
add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(odometry_node
  rclcpp geometry_msgs sensor_msgs nav_msgs tf2_ros std_msgs
)

# Add executable for velocity control node
add_executable(velocity_control_node src/velocity_control_node.cpp)
ament_target_dependencies(velocity_control_node
  rclcpp geometry_msgs sensor_msgs std_msgs
)

# Add executable for initial pose publisher
add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)
ament_target_dependencies(initial_pose_publisher
  rclcpp geometry_msgs tf2 tf2_ros
)

# Add executable for wall follow node
add_executable(wall_follow_node src/wall_follow_node.cpp)
ament_target_dependencies(wall_follow_node 
  rclcpp geometry_msgs sensor_msgs std_msgs
)

# Add executable and composable version for tf_relay to add map->f1tenth_1 from world->f1tenth_1
# when AMCL is disabled
add_executable(tf_relay_node src/tf_relay_node.cpp)
ament_target_dependencies(tf_relay_node
 rclcpp tf2_ros geometry_msgs
)

add_library(tf_relay_node_composable SHARED src/tf_relay_lib.cpp)
ament_target_dependencies(tf_relay_node_composable 
 rclcpp tf2_ros geometry_msgs rclcpp_components
)

rclcpp_components_register_nodes(tf_relay_node_composable "tf_relay_node_namespace::TfRelayNode")


# Install Cpp executables
install(TARGETS
  velocity_control_node
  initial_pose_publisher
  wall_follow_node
  odometry_node
  tf_relay_node
  tf_relay_node_composable
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
