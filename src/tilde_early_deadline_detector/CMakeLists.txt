cmake_minimum_required(VERSION 3.8)
project(tilde_early_deadline_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(INCLUDE_DIR
  include
  ${PROJECT_SOURCE_DIR}/include
)

include_directories("${INCLUDE_DIR}")
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(tilde_msg REQUIRED)
find_package(tilde_deadline_detector REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(tilde_early_deadline_detector_node SHARED
  src/forward_estimator.cpp
  src/tilde_early_deadline_detector_node.cpp)
ament_target_dependencies(tilde_early_deadline_detector_node
  rclcpp
  rclcpp_components
  tilde_deadline_detector
  tilde_msg)

rclcpp_components_register_node(tilde_early_deadline_detector_node
  PLUGIN "tilde_early_deadline_detector::TildeEarlyDeadlineDetectorNode"
  EXECUTABLE tilde_early_deadline_detector_node_exe)




install(TARGETS
  tilde_early_deadline_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
