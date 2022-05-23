cmake_minimum_required(VERSION 3.8)
project(tilde_deadline_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tilde_msg REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(tilde_deadline_detector_node SHARED
  src/forward_estimator.cpp
  src/tilde_deadline_detector_node.cpp)
ament_target_dependencies(tilde_deadline_detector_node
  rclcpp
  rclcpp_components
  tilde_msg)

rclcpp_components_register_node(tilde_deadline_detector_node
  PLUGIN "tilde_deadline_detector::TildeDeadlineDetectorNode"
  EXECUTABLE tilde_deadline_detector_node_exe)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(tilde REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  ament_add_gtest(test_forward_estimator
    src/forward_estimator.cpp
    test/test_forward_estimator.cpp)
  ament_target_dependencies(test_forward_estimator
    rclcpp
    tilde_msg)

  ament_add_gtest(test_tilde_deadline_detector_node
    test/test_tilde_deadline_detector_node.cpp)
  ament_target_dependencies(test_tilde_deadline_detector_node
    rclcpp
    builtin_interfaces
    sensor_msgs
    tilde
    tilde_msg)
  target_link_libraries(test_tilde_deadline_detector_node
    tilde_deadline_detector_node)

endif()

install(TARGETS
  tilde_deadline_detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
