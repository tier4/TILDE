# tilde_deadline_detector

## Description

Deadline detector by TILDE MessageTrackingTag.
You can detect deadlines in some scenarios:

- from the sensor to some node
- a specific path

## Requirement

- ROS 2 galactic
- TILDE enabled application
- The Messages should have the `header.stamp` field.

## Build

Do `colcon build`. We recommend the "Release" build for performance.

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Run

Use `ros2 run` as below.

```bash
$ ros2 run tilde_deadline_detector tilde_deadline_detector_node_exe \
    --ros-args --params-file src/tilde_deadline_detector/autoware_sensors.yaml \
```

By default, it prints subscribed topics, then runs silently.
The deadline notification is not implemented yet.

Here is a set of parameters.

| name                | about                                                                               |
| ------------------- | ----------------------------------------------------------------------------------- |
| `ignore_topics`     | don't subscribe these topics                                                        |
| `sensor_topics`     | regard nodes as sensors if MessageTrackingTag has no input_infos or the topic is in this list. |
| `target_topics`     | topics of which you want to detect deadline                                         |
| `deadline_ms`       | list of deadline [ms] in `target_topics` order.                                     |
| `expire_ms`         | internal data lifetime                                                              |
| `cleanup_ms`        | timer period to cleanup internal data                                               |
| `print_report`      | whether to print internal data                                                      |
| `clock_work_around` | set true when your bag file does not have `/clock`                                  |
| `show_performance`  | set true to show performance report                                                 |

This is a sample parameter yaml file.

```text
tilde_deadline_detector_node:
  ros__parameters:
    sensor_topics: [
      "/sensing/lidar/front_lower/self_cropped/pointcloud_ex",
      "/sensing/lidar/front_upper/self_cropped/pointcloud_ex",
      "/sensing/lidar/left_lower/self_cropped/pointcloud_ex",
      "/sensing/lidar/left_upper/self_cropped/pointcloud_ex",
      "/sensing/lidar/rear_lower/self_cropped/pointcloud_ex",
      "/sensing/lidar/rear_upper/self_cropped/pointcloud_ex",
      "/sensing/lidar/right_lower/self_cropped/pointcloud_ex",
      "/sensing/lidar/right_upper/self_cropped/pointcloud_ex",
      "/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias",
    ]
    target_topics: [
      /localization/pose_twist_fusion_filter/kinematic_state
    ]
    # specify deadline ms for topics in target_topics order.
    # 0 means no deadline, and negative values are replaced by 0
    deadline_ms: [
      0,
    ]
```
