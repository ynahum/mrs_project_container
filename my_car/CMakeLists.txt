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
  rclpy 
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  nav_msgs
  pluginlib
  behaviortree_cpp_v3)

  find_package(${PACKAGE} REQUIRED)

endforeach()


include_directories(
  include
)

add_library(debug_logger
  src/debug_logger.cpp
)

target_include_directories(debug_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(debug_logger rclcpp)

ament_export_include_directories(include)
ament_export_libraries(debug_logger)
ament_export_dependencies(rclcpp)

# Install the target
install(TARGETS debug_logger
  EXPORT export_debug_logger
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export the target
install(EXPORT export_debug_logger
  NAMESPACE my_car::
  DESTINATION share/my_car/cmake
)

ament_export_targets(export_debug_logger HAS_LIBRARY_TARGET)

install(DIRECTORY include/
  DESTINATION include
)


# Add executable for initial pose publisher
add_executable(path_publisher src/path_publisher.cpp)
ament_target_dependencies(path_publisher rclcpp geometry_msgs tf2 tf2_ros nav_msgs)
install(TARGETS path_publisher DESTINATION lib/${PROJECT_NAME})


  # Add executable for velocity control node
add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(odometry_node rclcpp geometry_msgs sensor_msgs nav_msgs tf2_ros std_msgs)
install(TARGETS odometry_node DESTINATION lib/${PROJECT_NAME})



  # Add executable for velocity control node
add_executable(velocity_control_node src/velocity_control_node.cpp)
target_link_libraries(velocity_control_node debug_logger)
ament_target_dependencies(velocity_control_node rclcpp geometry_msgs sensor_msgs std_msgs)
install(TARGETS velocity_control_node DESTINATION lib/${PROJECT_NAME})



# Add executable for initial pose publisher
add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)
ament_target_dependencies(initial_pose_publisher rclcpp geometry_msgs tf2 tf2_ros)
install(TARGETS initial_pose_publisher DESTINATION lib/${PROJECT_NAME})



  # Add executable for wall follow node
add_executable(wall_follow_node src/wall_follow_node.cpp)
ament_target_dependencies(wall_follow_node rclcpp geometry_msgs sensor_msgs std_msgs)
install(TARGETS wall_follow_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch maps behavior_trees DESTINATION share/${PROJECT_NAME})

ament_package()
