cmake_minimum_required(VERSION 3.5)
project(noise_cpp_pck)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Executable
add_executable(noise_cpp_node src/noise_cpp_node.cpp)
ament_target_dependencies(noise_cpp_node rclcpp std_msgs)

# Install
install(TARGETS
  noise_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()