# TILDE Samples

## About

Sample classes for TILDE.

## Nodes

As TILDE uses header.stamp, we have publishers/subscriptions with and without a header field.

| class                       | about                                                              |
| --------------------------- | ------------------------------------------------------------------ |
| SamplePublisherWithStamp    | Publish message with standard header (PointCloud2)                 |
| SamplePublisherWithoutStamp | Publish message without standard header (String)                   |
| RelayTimer                  | Subscribe PointCloud2 and String, and relay them                   |
| RelayTimerWithBuffer        | Similar with RelayTimer, but it has buffers and calls explicit API |

## Run

### (1) Publisher and Subscription with standard header

```bash
$ ros2 launch tilde_sample publisher_relay_with_header.launch.py
(snip)
[publisher_with_stamp-1] [INFO] [1646122380.746137604] [talker_with_stamp]: Publishing PointCloud2: 7 stamp: 1646122380745891914
[relay_timer-2] [INFO] [1646122380.746543541] [relay_timer]: RelayTimer sub PointCloud2 seq: 7 stamp: 1646122380745891914
[relay_timer-2] [INFO] [1646122380.757507728] [relay_timer]: RelayTimer pub PointCloud2 seq: 7 stamp: 1646122380745891914
(snip)
```

See MessageTrackingTag by `ros2 topic echo <topic>/message_tracking_tag`.
In this example, you can see `MessageTrackingTag.output_info.stamp` is filled.

```bash
$ ros2 topic echo /relay_with_stamp/message_tracking_tag
header:
  stamp:
    sec: 1646122380
    nanosec: 757608873
  frame_id: ''
output_info:
  topic_name: /relay_with_stamp
  node_fqn: ''
  seq: 7
  pub_time:
    sec: 1646122380
    nanosec: 757615453
  pub_time_steady:
    sec: 6934823
    nanosec: 865332706
  has_header_stamp: true
  header_stamp:                     # matches stamp of seq 7 of relay_timer
    sec: 1646122380
    nanosec: 745891914
input_infos:
- topic_name: /topic_with_stamp
  sub_time:
    sec: 1646122380
    nanosec: 746446909
  sub_time_steady:
    sec: 6934823
    nanosec: 854165185
  has_header_stamp: true
  header_stamp:                    # matches stamp of seq 7 of talker_with_stamp
    sec: 1646122380
    nanosec: 745891914
---
```

### (2) Publisher and Subscription without standard header

```bash
$ ros2 launch tilde_sample publisher_relay_without_header.launch.py
[publisher_without_stamp-1] [INFO] [1646122584.868273517] [talker_without_stamp]: Publishing String: '3' at '1646122584868032019'
[relay_timer-2] [INFO] [1646122584.868610655] [relay_timer]: RelayTimer sub String seq: 3
[relay_timer-2] [INFO] [1646122584.882195449] [relay_timer]: RelayTimer pub String seq: 3
```

In this example, you can see `MessageTrackingTag.output_info.has_header` is false,
and `stamp` has no meaning.

```bash
ros2 topic echo /relay_without_stamp/message_tracking_tag
```

<details>
<summary>Result</summary>

```bash
header:
  stamp:
    sec: 1646122584
    nanosec: 882317785
  frame_id: ''
output_info:
  topic_name: /relay_without_stamp
  node_fqn: ''
  seq: 3
  pub_time:
    sec: 1646122584
    nanosec: 882326545
  pub_time_steady:
    sec: 6935027
    nanosec: 983860696
  has_header_stamp: false             # no stamp
  header_stamp:
    sec: 0
    nanosec: 0
input_infos:
- topic_name: /topic_without_stamp
  sub_time:
    sec: 1646122584
    nanosec: 868535499
  sub_time_steady:
    sec: 6935027
    nanosec: 970071208
  has_header_stamp: false             # no stamp
  header_stamp:
    sec: 0
    nanosec: 0
---
```

</details>

### (3) Multiple Publishers and Subscription

```bash
ros2 launch tilde_sample multi_publisher_relay.launch.py
```

In this example, two publishers are launched.
One has the standard header field, and another doesn't.

RelayTimer is launched for the subscription, and relays
both publisher messages.

You can see:

- MessageTrackingTag has multiple input_info field

#### multiple input + with stamp

```bash
$ ros2 topic echo /relay_with_stamp/message_tracking_tag
[publisher_with_stamp-1] [INFO] [1646122873.266387175] [talker_with_stamp]: Publishing PointCloud2: 5 stamp: 1646122873266132674
[relay_timer-3] [INFO] [1646122873.266819191] [relay_timer]: RelayTimer sub PointCloud2 seq: 5 stamp: 1646122873266132674
[publisher_without_stamp-2] [INFO] [1646122873.270087090] [talker_without_stamp]: Publishing String: '5' at '1646122873269881680'
[relay_timer-3] [INFO] [1646122873.270392454] [relay_timer]: RelayTimer sub String seq: 5
[relay_timer-3] [INFO] [1646122873.287324103] [relay_timer]: RelayTimer pub PointCloud2 seq: 5 stamp: 1646122873266132674
[relay_timer-3] [INFO] [1646122873.287641253] [relay_timer]: RelayTimer pub String seq: 5
```

Result

```bash
header:
  stamp:
    sec: 1646122873
    nanosec: 287424898
  frame_id: ''
output_info:
  topic_name: /relay_with_stamp
  node_fqn: ''
  seq: 5
  pub_time:
    sec: 1646122873
    nanosec: 287431161
  pub_time_steady:
    sec: 6935316
    nanosec: 380230053
  has_header_stamp: true
  header_stamp:               # matches stamp of relay_timer seq 5
    sec: 1646122873
    nanosec: 266132674
input_infos:
- topic_name: /topic_with_stamp
  sub_time:
    sec: 1646122873
    nanosec: 266725249
  sub_time_steady:
    sec: 6935316
    nanosec: 359525561
  has_header_stamp: true
  header_stamp:              # matches stamp of talker_with_stamp seq 5
    sec: 1646122873
    nanosec: 266132674
- topic_name: /topic_without_stamp
  sub_time:
    sec: 1646122873
    nanosec: 270354306
  sub_time_steady:
    sec: 6935316
    nanosec: 363153994
  has_header_stamp: false    # no stamp
  header_stamp:
    sec: 0
    nanosec: 0
```

#### multiple input + without stamp

Now, let's see `/relay_without_stamp` MessageTrackingTag

```bash
ros2 topic echo /relay_without_stamp/message_tracking_tag
```

Result:

```bash
header:
  stamp:
    sec: 1646122966
    nanosec: 781916993
  frame_id: ''
output_info:
  topic_name: /relay_without_stamp
  node_fqn: ''
  seq: 2
  pub_time:
    sec: 1646122966
    nanosec: 781919913
  pub_time_steady:
    sec: 6935409
    nanosec: 871887129
  has_header_stamp: false           # no stamp
  header_stamp:
    sec: 0
    nanosec: 0
input_infos:
- topic_name: /topic_with_stamp
  sub_time:
    sec: 1646122966
    nanosec: 768327652
  sub_time_steady:
    sec: 6935409
    nanosec: 858296849
  has_header_stamp: true
  header_stamp:
    sec: 1646122966
    nanosec: 767682758
- topic_name: /topic_without_stamp
  sub_time:
    sec: 1646122966
    nanosec: 771331749
  sub_time_steady:
    sec: 6935409
    nanosec: 861299976
  has_header_stamp: false
  header_stamp:
    sec: 0
    nanosec: 0
```

### (4) Multiple Publishers and buffered Subscription

```bash
ros2 launch tilde_sample multi_publisher_buffered_relay.launch.py
```

In this example, two publishers are launched as in (3).

For the subscription side, RelayTimerWithBuffer is launched.

As RelayTimerWithBuffer uses explicit API,
you can see MessageTrackingTag has single input_info field.

### buffer with stamp

```bash
$ ros2 topic echo /relay_buffer_with_stamp/message_tracking_tag
header:
  stamp:
    sec: 1646123155
    nanosec: 485587145
  frame_id: ''
output_info:
  topic_name: /relay_buffer_with_stamp
  node_fqn: ''
  seq: 5
  pub_time:
    sec: 1646123155
    nanosec: 485593383
  pub_time_steady:
    sec: 6935598
    nanosec: 569846981
  has_header_stamp: true
  header_stamp:
    sec: 1646123155
    nanosec: 472929593
input_infos:                       # only one entry
- topic_name: /topic_with_stamp
  sub_time:
    sec: 1646123155
    nanosec: 473451355
  sub_time_steady:
    sec: 6935598
    nanosec: 557705720
  has_header_stamp: true
  header_stamp:
    sec: 1646123155
    nanosec: 472929593
```

### buffer without stamp

We cannot call explicit API because this topic has no `header.stamp`.
So, there are two input info entries.

```bash
$ ros2 topic echo /relay_buffer_without_stamp/message_tracking_tag
header:
  stamp:
    sec: 1646123029
    nanosec: 102143530
  frame_id: ''
output_info:
  topic_name: /relay_buffer_without_stamp
  node_fqn: ''
  seq: 9
  pub_time:
    sec: 1646123029
    nanosec: 102146186
  pub_time_steady:
    sec: 6935472
    nanosec: 190226558
  has_header_stamp: false
  header_stamp:
    sec: 0
    nanosec: 0
input_infos:                          # multiple input because explicit API is not called
- topic_name: /topic_with_stamp
  sub_time:
    sec: 1646123029
    nanosec: 88254671
  sub_time_steady:
    sec: 6935472
    nanosec: 176335881
  has_header_stamp: true
  header_stamp:
    sec: 1646123029
    nanosec: 87745929
- topic_name: /topic_without_stamp
  sub_time:
    sec: 1646123029
    nanosec: 78947692
  sub_time_steady:
    sec: 6935472
    nanosec: 167029850
  has_header_stamp: false
  header_stamp:
    sec: 0
    nanosec: 0
```
