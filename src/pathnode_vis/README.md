pathnode_vis
====

# Description

Sample application which uses topic info

# Requirement

ros2 galactic

# Usage

To use this package, you need the system which use pathnode::SubTimingAdvertiseNode instead of rclcpp::Node.
Also you should call `create_timing_advertise_subscription` and `create_timing_advertise_publisher`.

You can see callback latency and e2e latency on Filter-type graph by this package.

Here are some examples. We use customized autoware.proj logging simulator where we use pathnode at `/sensing` module.

(1) Print subscribers topic info.

There are five topics, so pathnode_vis shows four per-callback latencies(from some subscription callback to the nexe subscription callback) and e2e latency.

```bash
$ ros2 run pathnode_vis pathnode_vis

/sensing/lidar/top/self_cropped/pointcloud_ex -> /sensing/lidar/top/rectified/pointcloud_ex -> /sensing/lidar/top/outlier_filtered/pointcloud -> /sensing/lidar/concatenated/pointcloud -> /sensing/lidar/measurement_range_cropped/pointcloud
max:  65.2   39.9   15.0   10.0
avg:  53.0   28.5    9.0    5.5
min:  40.0   19.9    5.0    0.0
e2e: 110.0   96.0   80.0

max:  60.0   35.0   20.0   10.0
avg:  55.0   24.0    9.6    5.4
min:  50.0   19.7    5.0    0.0
e2e: 110.0   94.0   80.0
```

(2) print publisher topic info

In the publisher topic info, timestamp means published topic header timestamp.
/sensing nodes conveys the lidar timestamp, we get latency is 0.0.

```
$ ros2 run pathnode_vis pathnode_vis --ros-args -p watches_pub:=True

/sensing/lidar/top/self_cropped/pointcloud_ex -> /sensing/lidar/top/mirror_cropped/pointcloud_ex -> /sensing/lidar/top/outlier_filtered/pointcloud -> /sensing/lidar/concatenated/pointcloud -> /sensing/lidar/measurement_range_cropped/pointcloud
max:   0.0    0.0    0.0    0.0
avg:   0.0    0.0    0.0    0.0
min:   0.0    0.0    0.0    0.0
e2e:   0.0    0.0    0.0

max:   0.0    0.0    0.0    0.0
avg:   0.0    0.0    0.0    0.0
min:   0.0    0.0    0.0    0.0
e2e:   0.0    0.0    0.0
```

You can set log level by `LOG_LEVEL=DEBUG`.
See source codes for parameters.
