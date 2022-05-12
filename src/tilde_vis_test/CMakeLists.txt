cmake_minimum_required(VERSION 3.8)
project(tilde_vis_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(tilde REQUIRED)

add_library(tilde_vis_test SHARED
  src/tilde_vis_test.cpp)
ament_target_dependencies(tilde_vis_test
    "rclcpp"
    "rclcpp_components"
    "std_msgs"
    "tilde"
    "sensor_msgs")

rclcpp_components_register_node(tilde_vis_test
  PLUGIN "tilde_vis_test::Sensor"
  EXECUTABLE sensor)
rclcpp_components_register_node(tilde_vis_test
  PLUGIN "tilde_vis_test::Relay"
  EXECUTABLE relay)
rclcpp_components_register_node(tilde_vis_test
  PLUGIN "tilde_vis_test::Filter"
  EXECUTABLE filter)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS tilde_vis_test EXPORT tilde_vis_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
