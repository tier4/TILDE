# tilde_vis

## Description

Sample application which uses topic info or MessageTrackingTag.

## Requirement

- ros2 galactic
- tilde enabled application

## Applications

To use this package, you need the system which uses tilde::TildeNode instead of rclcpp::Node.
Also you should call `create_tilde_subscription` and `create_tilde_publisher`.

You can use some tools.

- (1) latency viewer: online latency viewer
  - run your application
  - run `latency_viewer`
- (2) parse_message_tracking_tag & ipython: analyze latency from rosbag
  - run your application and record rosbag2
  - run `parse_message_tracking_tag` to get MessageTrackingTag pkl.
  - run ipython and do what you want
- (3) tilde_vis (not well maintained)
  - save `/*/message_tracking_tag` to rosbag
  - then, run `tilde_vis`
- (4) visualization by CARET
  - you can visualize message flows by CARET jupyter notebook

## (1) latency viewer

### About

The latency viewer outputs E2E latency online, just like `vmstat` or `top`.

### Run

Run the latency viewer **after** running the target application.
Latency viewer grasps your application topic graph in constructor by discovering and listening `MessageTrackingTag` type topics.

Latency viewer shows latencies from `target_topic` to source topics.
Here is the example command.

```bash
# (1) run your application
ros2 launch ...

# (2) run latency viewer
ros2 run tilde_vis latency_viewer \
  --ros-args \
    -p target_topic:=/localization/pose_twist_fusion_filter/twist_with_covariance
```

In default, you see output just like `top`. Internally [curses](https://docs.python.org/3/library/curses.html) is used.
To save the results to a file, use `--batch` option and redirect or `tee` stdout.

```bash
ros2 run tilde_vis latency_viewer --batch ...
```

### Parameters

#### General parameters

| name                     | values              | about                                      |
| ------------------------ | ------------------- | ------------------------------------------ |
| `mode`                   | `one_hot` or `stat` | Calculation rule. See below.               |
| `wait_sec_to_init_graph` | positive integer    | Initial wait seconds to grasp topic graph. |

- mode
  - one_hot: prints the specific flow latency
  - stat: calculates statistics per second

#### topic graph specific parameters

You can use topic graph specific options.

| name              | about                                                                           |
| ----------------- | ------------------------------------------------------------------------------- |
| `excludes_topics` | Topics you don't want to watch, such as `debug` topic.                          |
| `stops`           | Stop traversal if latency viewer reaches these topics to prevent infinite loop. |

Additionally, you can specify "skip" setting to skip non-TILDE node.
But you need to directly edit `LatencyViewerNode.init_skips()` to do it.

### Result

You can see the following lines are displayed repeatedly.
Columns are "topic name", "header stamp", "latency in ROS time", "latency in steady time".

```text
/control/trajectory_follower/lateral/control_cmd                                 1618559274.549348133      0      0
 /localization/twist                                                             1618559274.544330488      5      3
 /planning/scenario_planning/trajectory                                          1618559274.479350019     15     12
  (snip)
    /sensing/lidar/concatenated/pointcloud                                       1618559273.951109000    289    288
     /sensing/lidar/left/outlier_filtered/pointcloud                             1618559274.168087000    305    302
      /sensing/lidar/left/mirror_cropped/pointcloud_ex                           1618559274.168087000    305    304
       /sensing/lidar/left/self_cropped/pointcloud_ex                            1618559274.168087000    305    304
        /sensing/lidar/left/pointcloud_raw_ex*                                                     NA     NA     NA
     /sensing/lidar/right/outlier_filtered/pointcloud                            1618559274.170810000    300    301
      /sensing/lidar/right/mirror_cropped/pointcloud_ex                          1618559274.170810000    305    302
       /sensing/lidar/right/self_cropped/pointcloud_ex                           1618559274.170810000    305    303
        /sensing/lidar/right/pointcloud_raw_ex*                                                    NA     NA     NA
     /sensing/lidar/top/outlier_filtered/pointcloud                              1618559273.951109000    334    333
      /sensing/lidar/top/mirror_cropped/pointcloud_ex                            1618559273.951109000    410    407
       /sensing/lidar/top/self_cropped/pointcloud_ex                             1618559273.951109000    435    433
        /sensing/lidar/top/pointcloud_raw_ex*                                                      NA     NA     NA
     /vehicle/status/twist*                                                                        NA     NA     NA
```

See [latency_viewer.md](../../doc/latency_viewer.md) in detail (in Japanese).

### key binding in ncurses view

In ncurses view, you can use interactive commands.
When you enter "move" or "filter" commands, periodic update of view stops.
Hit

| keys                | about                                                 |
| ------------------- | ----------------------------------------------------- |
| down/up arrow, j, k | move lines.                                           |
| d, u                | page up/down.                                         |
| g, G                | move to top or bottom of lines                        |
| r, /                | filter lines by regex. Hit Enter for periodic update. |
| q, F                | stop "move" mode and restart periodic update          |

## (2) parse_message_tracking_tag

### About

Read rosbag2 file, and create pkl file.

### Run

```bash
ros2 run tilde_vis parse_message_tracking_tag <path/to/rosbag/db_file>
```

`topic_infos.pkl` is created.
You can get `message_tracking_tag.MessageTrackingTags` by reading this pkl file.

## (3) tilde_vis

Here are some examples. We use customized autoware.proj logging simulator where we use tilde at `/sensing` module.

(1) Print subscribers topic info.

There are five topics, so tilde_vis shows four per-callback latencies(from some subscription callback to the next subscription callback) and e2e latency.

```bash
$ ros2 run tilde_vis tilde_vis

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

```bash
$ ros2 run tilde_vis tilde_vis --ros-args -p watches_pub:=True

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

## (4) visualization by CARET

You can draw message flows from a rosbag file.
Run tilde_vis/caret_vis_tilde in a CARET jupyter notebook.
