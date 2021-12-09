pathnode_vis
====

# Description

Sample application which uses topic info

# Requirement

ros2 galactic

# Usage

## General Usage

To use this package, you need the system which uses pathnode::SubTimingAdvertiseNode instead of rclcpp::Node.
Also you should call `create_timing_advertise_subscription` and `create_timing_advertise_publisher`.

You can use some tools.

- (1) offline analysis tool
  - save `/*/info/pub` to rosbag
  - then, run `pathnode_vis`
- (2) online analysis tool
  - run logging simulator
  - run `latency_viewer`

PubInfo contains the "source" information about the published topic. PubInfo contains:

- output_info: about published message information, including the topic name, publishing timestamp, and header stamp.
- input_info: about the "source" information to create the published message, including topic names, subscribed timestamps, and header stamps.

## Advanced API

In general, there are two types of nodes.

- (1) no buffer
  - node subscribes to one or more topics.
  - when node subscribe topic, it holds only one generation, just like `this->msg_ = msg` in the subscription callback
- (2) with buffer
  - node subscribes to one or more topics.
  - node has many generations, just like `this->msg_buffer_.push_back(msg)` in the subscription callback

TILDE assumes "nodes make publishing message by using the latest subscribed message".
So TILDE cannot handle type (2) node well in default.

In such a case, you can use `add_explicit_input_info`.

``` cpp
void timercallback()
{
  // select from msg1_buffer
  auto msg1 = take_appropriate_msg(msg1_buffer);

  // tell the publisher that "we use this header.stamp on msg1 topic"
  pub->add_explicit_input_info(msg1_sub->get_topic_name(), msg1->header.stamp);
  
  // then publish output
  auto output = do_something(msg1, ...);
  pub->publish(output);
}
```

Be aware you need to call `add_explicit_input_info` to the referenced topics.

# Example

## pathnode_vis

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

## latency viewer

The latency viewer outputs E2E latency online, just like `vmstat` or `top`.
You can run the latency viewer before or after running the target application.

The latency viewer show latencies from `target_topic` to source topics.
Here is the example command and report.

``` bash
# (1) run your application
ros2 launch ...

# (2) run latency viewer
ros2 run pathnode_vis latency_viewer \
  --ros-args \
    -p target_topic:=/localization/pose_twist_fusion_filter/twist_with_covariance
```

You can see the following lines are displayed repeatedly.

```
topic                                                                               dur    dur    dur    e2e    e2e    e2e
/localization/pose_twist_fusion_filter/twist_with_covariance                        0.0    0.0    0.0    0.0    0.0    0.0 False
/localization/pose_estimator/pose                                                 232.0  289.0  354.0    4.0   51.8  124.0 False
/localization/pose_estimator/pose_with_covariance                                 232.0  289.0  354.0    4.0   51.8  124.0 False
(snip)
/sensing/imu/imu_data                                                               0.0   19.2   71.0    4.0   21.3   74.0 True
/vehicle/status/twist                                                               5.0   35.9   75.0    9.0   38.5   79.0 True
(snip)
/sensing/lidar/top/rectified/pointcloud                                           232.0  289.1  354.0   30.0   89.1  160.0 True
```

This means:

- `topic` columns:
  - topics are displayed from `target_topic` to source topics, such as sensors.
  - latency viewer traverses your application DAG and finds these topics.
  - Topics forms DAG in general, and topics are listed by BFS order from `target_topic`.
- dur and e2e columns:
  - e2e column may be what you want. This is the end-to-end latency from listed topics to `target_topic`.  
    There are three floating numbers, which mean min, mean, max in order.
    The definition is `<target_topic publising time> - <listed topic subscription time>)`.  
    So, you can see it takes 30.0 [ms] from pointcloud subscription to twist_with_covariance publishing in the fastest case.

Other parameter:

- `topics` [string array]
  - `/*/info/pub` topics list
- `wait_sec_to_init_graph` [int]
  - As in above, latency viewer constructs DAG of your application.
  - Latency viewr listens topics for `wait_sec_to_init_graph` seconds before constructing DAG.
- and so on
  - please see latency_viewer.py
