cmake_minimum_required(VERSION 3.8)
project(franka_cc_6)

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


# for the services
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(orocos_kdl REQUIRED)

# Generate service code (muse tbe ebfore add library)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeIK.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)
# ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Add your controller library
add_library(${PROJECT_NAME}_lib SHARED
  src/MyController_cppFile.cpp
)



target_compile_features(${PROJECT_NAME}_lib PUBLIC c_std_99 cxx_std_17)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Dependencies for the library
ament_target_dependencies(${PROJECT_NAME}_lib
  controller_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  Eigen3
  urdf
  kdl_parser
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_kdl
)


target_link_libraries(${PROJECT_NAME}_lib
  orocos-kdl
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# Export the plugin
pluginlib_export_plugin_description_file(controller_interface MyController_plugins.xml)

# Install the library
install(TARGETS ${PROJECT_NAME}_lib
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
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  controller_interface
  rosidl_default_runtime
  rclcpp
  rclcpp_lifecycle
  pluginlib
  Eigen3
)

ament_package()