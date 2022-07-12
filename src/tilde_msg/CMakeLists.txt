cmake_minimum_required(VERSION 3.8)
project(tilde_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/SubTopicTimeInfo.msg"
  "msg/PubTopicTimeInfo.msg"
  "msg/MessageTrackingTag.msg"
  "msg/TestTopLevelStamp.msg"
  "msg/Source.msg"
  "msg/DeadlineNotification.msg"

  "msg/SteeSource.msg"
  "msg/SteePointCloud2.msg"

  "msg/SteePolygonStamped.msg"
  "msg/SteePoseStamped.msg"
  "msg/SteePoseWithCovarianceStamped.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
