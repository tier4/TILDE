cmake_minimum_required(VERSION 3.8)
project(pathnode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)

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
ament_export_include_directories(include)
install(
  DIRECTORY include/
  DESTINATION include
  )

add_library(${PROJECT_NAME} SHARED
  "src/path_node.cpp"
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "PATHNODE_BUILDING_LIBRARY")
ament_export_targets(${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
