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
  std_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Create Cpp executable
add_executable(velocity_control_node src/velocity_control_node.cpp)
ament_target_dependencies(velocity_control_node 
  rclcpp geometry_msgs sensor_msgs std_msgs
)

# Install Cpp executables
install(TARGETS
  velocity_control_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
