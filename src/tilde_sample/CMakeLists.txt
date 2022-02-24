cmake_minimum_required(VERSION 3.8)
project(tilde_sample)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(tilde REQUIRED)
find_package(tilde_msg REQUIRED)

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

include_directories(include)

add_library(tilde_sample SHARED
  src/sample_publish_header.cpp
  src/sample_relay_with_sub_timing.cpp
  src/sample_multi_callback.cpp
  src/p2_sample_pub.cpp
  src/p2_relay_timer.cpp
  src/p2_relay_timer_with_buffer.cpp
  src/p2_goal.cpp)
ament_target_dependencies(tilde_sample
    "rclcpp"
    "rclcpp_components"
    "std_msgs"
    "tilde"
    "tilde_msg"
    "sensor_msgs")

rclcpp_components_register_node(tilde_sample
  PLUGIN "tilde_sample::Talker"
  EXECUTABLE talker)
rclcpp_components_register_node(tilde_sample
  PLUGIN "tilde_sample::TalkerWithHeader"
  EXECUTABLE talker_with_header)
rclcpp_components_register_node(tilde_sample
  PLUGIN "tilde_sample::Listener"
  EXECUTABLE listener)
rclcpp_components_register_node(tilde_sample
  PLUGIN "tilde_sample::Relay"
  EXECUTABLE relay)
rclcpp_components_register_node(tilde_sample
  PLUGIN "tilde_sample::RelayWithPath"
  EXECUTABLE relay_with_path)
rclcpp_components_register_node(tilde_sample
  PLUGIN "tilde_sample::RelayWithPath2"
  EXECUTABLE relay_with_path2)
rclcpp_components_register_node(tilde_sample
  PLUGIN "tilde_sample::RelayWithSubTiming"
  EXECUTABLE relay_with_sub_timing)
rclcpp_components_register_node(tilde_sample
  PLUGIN "tilde_sample::SampleMultiCallback"
  EXECUTABLE sample_multi_callback)

rclcpp_components_register_node(tilde_sample
  PLUGIN "tilde_sample::P2Publisher"
  EXECUTABLE p2_publisher)
rclcpp_components_register_node(tilde_sample
  PLUGIN "tilde_sample::P2RelayTimer"
  EXECUTABLE p2_relaytimer)
rclcpp_components_register_node(tilde_sample
  PLUGIN "tilde_sample::P2RelayTimerWithBuffer"
  EXECUTABLE p2_relaytimer_with_buffer)
rclcpp_components_register_node(tilde_sample
  PLUGIN "tilde_sample::P2Goal"
  EXECUTABLE p2_goal)


install(TARGETS
  tilde_sample)
install(
  TARGETS tilde_sample EXPORT tilde_sample
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()