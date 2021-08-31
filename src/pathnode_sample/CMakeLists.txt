cmake_minimum_required(VERSION 3.8)
project(pathnode_sample)

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

find_package(pathnode REQUIRED)

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

function(add_dependencies library)
  ament_target_dependencies(${library}
    "rclcpp"
    "rclcpp_components"
    "std_msgs"
    "pathnode"
    )
endfunction()

include_directories(include)

add_library(pathnode_sample SHARED
  src/sample_publisher.cpp
  src/sample_subscriber.cpp
  src/sample_relay.cpp
  src/sample_relay_with_path.cpp
  )
ament_target_dependencies(pathnode_sample
    "rclcpp"
    "rclcpp_components"
    "std_msgs"
    "pathnode"
    )
# target_link_libraries(pathnode_sample
#   "pathnode"
# )

rclcpp_components_register_node(pathnode_sample
  PLUGIN "pathnode_sample::Talker"
  EXECUTABLE talker)
rclcpp_components_register_node(pathnode_sample
  PLUGIN "pathnode_sample::Listener"
  EXECUTABLE listener)
rclcpp_components_register_node(pathnode_sample
  PLUGIN "pathnode_sample::Relay"
  EXECUTABLE relay)
rclcpp_components_register_node(pathnode_sample
  PLUGIN "pathnode_sample::RelayWithPath"
  EXECUTABLE relay_with_path)


install(TARGETS
  pathnode_sample)
install(
  TARGETS pathnode_sample EXPORT pathnode_sample
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
