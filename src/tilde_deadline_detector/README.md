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
    --ros-args --params-file src/tilde_deadline_detector/autoware_sensors.yaml
```

By default, it prints subscribed topics, then runs silently.
The deadline notification is not implemented yet.

Here is a set of parameters.

| category          | name                     | about                                                                                          |
| ----------------- | ------------------------ | ---------------------------------------------------------------------------------------------- |
| system input      | `sensor_topics`          | regard nodes as sensors if MessageTrackingTag has no input_infos or the topic is in this list. |
| ignore            | `ignore_topics`          | don't subscribe these topics                                                                   |
| skip              | `skips_main_in`          | skip setting: input main topics                                                                |
|                   | `skips_main_out`         | skip setting: output main topic, in `skips_main_in` order                                      |
| target & deadline | `target_topics`          | topics of which you want to detect deadline                                                    |
|                   | `deadline_ms`            | list of deadline [ms] in `target_topics` order.                                                |
| maintenance       | `expire_ms`              | internal data lifetime                                                                         |
|                   | `cleanup_ms`             | timer period to cleanup internal data                                                          |
| miscellaneous     | `clock_work_around`      | set true when your bag file does not have `/clock`                                             |
| debug print       | `show_performance`       | set true to show performance report                                                            |
|                   | `print_report`           | whether to print internal data                                                                 |
|                   | `print_pending_messages` | whether to print pending messages                                                              |

See [autoware_sensors.yaml](autoware_sensors.yaml) for a sample parameter yaml file.

## Notification

If the target topic overruns, `deadline_notification` topic is published.
The message type is [DeadlineNotification.msg](../tilde_msg/msg/DeadlineNotification.msg).
