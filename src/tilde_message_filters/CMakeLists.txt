cmake_minimum_required(VERSION 3.8)
project(tilde_message_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(tilde)
find_package(message_filters)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
ament_export_include_directories(include)

add_library(sample_tilde_message_filter SHARED
  src/sample_tilde_subscriber.cpp
  src/sample_synchronizer.cpp
  src/sample_sync_publisher.cpp)
ament_target_dependencies(sample_tilde_message_filter
  "tilde"
  "message_filters"
  "rclcpp_components"
  "std_msgs"
  "sensor_msgs")
rclcpp_components_register_node(sample_tilde_message_filter
  PLUGIN "sample_tilde_message_filter::SampleSubscriberWithHeader"
  EXECUTABLE subscriber_with_header)
rclcpp_components_register_node(sample_tilde_message_filter
  PLUGIN "sample_tilde_message_filter::SampleSynchronizer2"
  EXECUTABLE sample_synchronizer2)
rclcpp_components_register_node(sample_tilde_message_filter
  PLUGIN "sample_tilde_message_filter::SampleSynchronizer3"
  EXECUTABLE sample_synchronizer3)
rclcpp_components_register_node(sample_tilde_message_filter
  PLUGIN "sample_tilde_message_filter::SamplePublisherWithHeader"
  EXECUTABLE publisher_with_header)


install(TARGETS
  sample_tilde_message_filter)
install(
  TARGETS sample_tilde_message_filter EXPORT sample_tilde_message_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


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

ament_package()
