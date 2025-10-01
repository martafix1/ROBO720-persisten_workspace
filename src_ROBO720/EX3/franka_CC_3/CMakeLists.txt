cmake_minimum_required(VERSION 3.8)
project(franka_CC_3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)


# Add your controller library
add_library(${PROJECT_NAME} SHARED
  src/MyController_cppFile.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Dependencies for the library
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  Eigen3
  urdf
  kdl_parser
)

# Export the plugin
pluginlib_export_plugin_description_file(controller_interface MyController_plugins.xml)

# Install the library
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install plugin description
install(FILES MyController_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  Eigen3
)

ament_package()