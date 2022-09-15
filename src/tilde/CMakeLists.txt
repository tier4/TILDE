cmake_minimum_required(VERSION 3.8)
project(tilde)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tilde_msg REQUIRED)
find_package(rmw REQUIRED)
find_package(LTTngUST REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_tilde_publisher
    test/test_tilde_publisher.cpp)
  target_include_directories(test_tilde_publisher
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(test_tilde_publisher
    ${PROJECT_NAME})
  ament_target_dependencies(test_tilde_publisher
    "rclcpp"
    "tilde_msg")

  ament_add_gtest(test_tilde_node
    test/test_tilde_node.cpp)
  target_include_directories(test_tilde_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(test_tilde_node
    ${PROJECT_NAME})
  ament_target_dependencies(test_tilde_node
    "rclcpp"
    "tilde_msg"
    "sensor_msgs"
    "std_msgs")

  ament_add_gtest(test_stamp_processor
    test/test_stamp_processor.cpp)
  target_include_directories(test_stamp_processor
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(test_stamp_processor
    ${PROJECT_NAME})
  ament_target_dependencies(test_stamp_processor
    "rclcpp"
    "sensor_msgs"
    "std_msgs")

  ament_add_gtest(test_stee_node
    test/test_stee_node.cpp)
  target_include_directories(test_stee_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(test_stee_node
    ${PROJECT_NAME})
  ament_target_dependencies(test_stee_node
    "rclcpp"
    "tilde_msg"
    "sensor_msgs")

  ament_add_gtest(test_stee_source_table
    test/test_stee_source_table.cpp)
  target_include_directories(test_stee_source_table
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(test_stee_source_table
    ${PROJECT_NAME})
  ament_target_dependencies(test_stee_source_table
    "rclcpp"
    "tilde_msg"
    "sensor_msgs")

endif()

include_directories(include)
ament_export_include_directories(include)
install(
  DIRECTORY include/
  DESTINATION include)

add_library(${PROJECT_NAME} SHARED
  "src/tilde_node.cpp"
  "src/tilde_publisher.cpp"
  "src/stee_sources_table.cpp"
  "src/stee_node.cpp"
  "src/tp.c"
)
target_link_libraries(${PROJECT_NAME} ${LTTNGUST_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "tilde_msg"
  LTTngUST
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "TILDE_BUILDING_LIBRARY")
ament_export_targets(${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp tilde_msg)

add_library(stee_republisher_node SHARED
  src/stee_republisher_node_pointcloud2.cpp
  src/stee_republisher_node_imu.cpp
  src/stee_republisher_node_map.cpp)
ament_target_dependencies(stee_republisher_node
  "rclcpp_components")
target_link_libraries(stee_republisher_node
  ${PROJECT_NAME})
rclcpp_components_register_node(stee_republisher_node
  PLUGIN "tilde::SteeRepublisherNode"
  EXECUTABLE stee_republisher_node_pointcloud2_exe)
rclcpp_components_register_node(stee_republisher_node
  PLUGIN "tilde::SteeRepublisherNodeImu"
  EXECUTABLE stee_republisher_node_imu_exe)
rclcpp_components_register_node(stee_republisher_node
  PLUGIN "tilde::SteeRepublisherNodeMap"
  EXECUTABLE stee_republisher_node_map_exe)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS stee_republisher_node
  stee_republisher_node_pointcloud2_exe
  stee_republisher_node_imu_exe
  stee_republisher_node_map_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
