cmake_minimum_required(VERSION 3.8)
project(KinematicsEX1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# this to apply same dependencies to all nodes automatically and make it cleaner
foreach(target node1 node2)
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target}
    PUBLIC
      $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
      $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${target} rclcpp std_msgs)
endforeach()

# this one does not need eigen
add_executable(rnd_configurable src/node_rnd_config.cpp)
ament_target_dependencies(rnd_configurable rclcpp std_msgs)  # remove std_msgs if not used


install(TARGETS
  node1
  node2
  rnd_configurable
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
