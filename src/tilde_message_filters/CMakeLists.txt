cmake_minimum_required(VERSION 3.8)
project(tilde_message_filters)

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
find_package(tilde)
find_package(message_filters)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
ament_export_include_directories(include)
install(
  DIRECTORY include/
  DESTINATION include)

add_library(sample_tilde_message_filter SHARED
  src/sample_tilde_subscriber.cpp
  src/sample_tilde_synchronizer.cpp
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
  PLUGIN "sample_tilde_message_filter::SampleTildeSynchronizer2"
  EXECUTABLE sample_synchronizer2)
rclcpp_components_register_node(sample_tilde_message_filter
  PLUGIN "sample_tilde_message_filter::SampleTildeSynchronizer3"
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
  find_package(tilde)
  find_package(message_filters REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cpplint
    ament_cmake_cppcheck
  )
  ament_lint_auto_find_test_dependencies()

  # we skip cpplint of borrowed files
  find_package(ament_cmake_cpplint)
  ament_cpplint(EXCLUDE
    "test/test_from_original_subscriber.cpp"
    "test/test_from_original_synchronizer.cpp")

  # TODO(y-okumura-isp): syntaxError but success to build & UT
  find_package(ament_cmake_cppcheck)
  ament_cppcheck(EXCLUDE
    "tilde_subscriber.hpp"
  )

  ament_add_gtest(test_tilde_subscriber
    test/test_tilde_subscriber.cpp)
    target_include_directories(test_tilde_subscriber
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(test_tilde_subscriber
    "tilde"
    "message_filters"
    "sensor_msgs"
    "std_msgs")

  ament_add_gtest(test_tilde_synchronizer
    test/test_tilde_synchronizer.cpp)
    target_include_directories(test_tilde_synchronizer
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(test_tilde_synchronizer
    "tilde"
    "message_filters"
    "sensor_msgs"
    "std_msgs")

  ament_add_gtest(test_from_original_subscriber
    test/test_from_original_subscriber.cpp)
    target_include_directories(test_from_original_subscriber
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(test_from_original_subscriber
    "tilde"
    "message_filters"
    "rclcpp_lifecycle"
    "sensor_msgs"
    "std_msgs")

  ament_add_gtest(test_from_original_synchronizer
    test/test_from_original_synchronizer.cpp)
    target_include_directories(test_from_original_synchronizer
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(test_from_original_synchronizer
    "tilde"
    "message_filters"
    "sensor_msgs"
    "std_msgs")
endif()

ament_package()
