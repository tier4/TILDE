# TILDE Samples

## About

Sample classes for TILDE.

## Nodes

As TILDE uses header.stamp, we have publishers/subscriptions with and without a header field.

| class                       | about                                                              |
|-----------------------------|--------------------------------------------------------------------|
| SamplePublisherWithStamp    | Publish message with standard header (PointCloud2)                 |
| SamplePublisherWithoutStamp | Publish message without standard header (String)                   |
| RelayTimer                  | Subscribe PointCloud2 and String, and relay them                   |
| RelayTimerWithBuffer        | Similar with RelayTimer, but it has buffers and calls explicit API |

## Run

### (1) Publisher and Subscription with standard header

``` bash
ros2 launch tilde_sample publisher_relay_with_header.launch.py
```

See PubInfo by `ros2 topic echo <topic>/info/pub`.
In this example, you can see `PubInfo.outpub_info.stamp` is filled.

``` bash
$ ros2 topic echo /relay_with_stamp/info/pub

header:
  stamp:
    sec: 1646031805
    nanosec: 644434606
  frame_id: ''
output_info:
  topic_name: /relay_with_stamp
  node_fqn: ''
  seq: 18
  pub_time:
    sec: 1646031805
    nanosec: 644441861
  pub_time_steady:
    sec: 6844251
    nanosec: 483557559
  header_stamp:
    sec: 1646031805
    nanosec: 617353429
input_infos:
- topic_name: /topic_with_stamp
  sub_time:
    sec: 1646031805
    nanosec: 617838735
  sub_time_steady:
    sec: 6844251
    nanosec: 456956382
  has_header_stamp: true
  header_stamp:
    sec: 1646031805
    nanosec: 617353429
```

### (2) Publisher and Subscription without standard header

``` bash
ros2 launch tilde_sample publisher_relay_without_header.launch.py
```

In this example, you can see `PubInfo.outpub_info.has_header` is false,
and `stamp` has no meaning.

``` bash
$ ros2 topic echo /relay_without_stamp/info/pub
```

<details>
<summary>Result</summary>

```
header:
  stamp:
    sec: 1646034075
    nanosec: 142661059
  frame_id: ''
output_info:
  topic_name: /relay_without_stamp
  node_fqn: ''
  seq: 13
  pub_time:
    sec: 1646034075
    nanosec: 142667675
  pub_time_steady:
    sec: 6846520
    nanosec: 913127491
  header_stamp:
    sec: 1646034075
    nanosec: 142651785
input_infos:
- topic_name: /topic_without_stamp
  sub_time:
    sec: 1646034075
    nanosec: 131743819
  sub_time_steady:
    sec: 6846520
    nanosec: 902204465
  has_header_stamp: false    # false
  header_stamp:
    sec: 0
    nanosec: 0
```
</details>

### (3) Multiple Publishers and Subscription

``` bash
ros2 launch tilde_sample multi_publisher_relay.launch.py
```

In this example, two publshers are launched.
One has the standard header field, and another doesn't.

RelayTimer is launched for the subscription, and relays
both publisher messages.

You can see:
- PubInfo has multiple input_info field

#### multiple input + with stamp

``` bash
$ ros2 topic echo /relay_with_stamp/info/pub
```

Result

```
header:
  stamp:
    sec: 1646034896
    nanosec: 212029258
  frame_id: ''
output_info:
  topic_name: /relay_with_stamp
  node_fqn: ''
  seq: 95
  pub_time:
    sec: 1646034896
    nanosec: 212035964
  pub_time_steady:
    sec: 6847341
    nanosec: 957675931
  header_stamp:
    sec: 1646034896
    nanosec: 199425675
input_infos:
- topic_name: /topic_with_stamp
  sub_time:
    sec: 1646034896
    nanosec: 199925450
  sub_time_steady:
    sec: 6847341
    nanosec: 945566700
  has_header_stamp: true
  header_stamp:
    sec: 1646034896
    nanosec: 199425675
- topic_name: /topic_without_stamp
  sub_time:
    sec: 1646034896
    nanosec: 195290738
  sub_time_steady:
    sec: 6847341
    nanosec: 940932624
  has_header_stamp: false
  header_stamp:
    sec: 0
    nanosec: 0
```

#### multiple input + without stamp

Now, let's see `/relay_without_stamp` PubInfo

``` bash
$ ros2 topic echo /relay_without_stamp/info/pub
```

Result:

```
header:
  stamp:
    sec: 1646034849
    nanosec: 210780402
  frame_id: ''
output_info:
  topic_name: /relay_without_stamp
  node_fqn: ''
  seq: 48
  pub_time:
    sec: 1646034849
    nanosec: 210782813
  pub_time_steady:
    sec: 6847294
    nanosec: 957841587
  header_stamp:            # no header stamp
    sec: 1646034849
    nanosec: 210778123
input_infos:
- topic_name: /topic_with_stamp
  sub_time:
    sec: 1646034849
    nanosec: 198490103
  sub_time_steady:
    sec: 6847294
    nanosec: 945550375
  has_header_stamp: true
  header_stamp:
    sec: 1646034849
    nanosec: 198009021
- topic_name: /topic_without_stamp
  sub_time:
    sec: 1646034849
    nanosec: 193974176
  sub_time_steady:
    sec: 6847294
    nanosec: 941034630
  has_header_stamp: false
  header_stamp:
    sec: 0
    nanosec: 0
```

### (4) Multiple Publishers and buffered Subscription

``` bash
ros2 launch tilde_sample multi_publisher_buffered_relay.launch.py
```

In this example, two publshers are launched as in (3).

For the subscription side, RelayTimerWithBuffer is launched.

As RelayTimerWithBuffer uses explicit API, 
you can see PubInfo has single input_info field.

### buffer with stamp

``` bash
$ ros2 topic echo /relay_buffer_with_stamp/info/pub
header:
  stamp:
    sec: 1646095847
    nanosec: 467303827
  frame_id: ''
output_info:
  topic_name: /relay_buffer_with_stamp
  node_fqn: ''
  seq: 19
  pub_time:
    sec: 1646095847
    nanosec: 467312428
  pub_time_steady:
    sec: 6908291
    nanosec: 375924215
  header_stamp:
    sec: 1646095846
    nanosec: 456617452
input_infos:                          # only one entry
- topic_name: /topic_with_stamp
  sub_time:
    sec: 1646095846
    nanosec: 467486614
  sub_time_steady:
    sec: 6908290
    nanosec: 376127658
  has_header_stamp: true
  header_stamp:
    sec: 1646095846
    nanosec: 456617452
```

### buffer without stamp

We cannot call explicit API because this topic has no `header.stamp`.
So, there are two input info entries.

``` bash
$ ros2 topic echo /relay_buffer_without_stamp/info/pub
header:
  stamp:
    sec: 1646095910
    nanosec: 469364604
  frame_id: ''
output_info:
  topic_name: /relay_buffer_without_stamp
  node_fqn: ''
  seq: 82
  pub_time:
    sec: 1646095910
    nanosec: 469367381
  pub_time_steady:
    sec: 6908354
    nanosec: 376072157
  header_stamp:
    sec: 1646095910
    nanosec: 469361798
input_infos:                        # multiple input because explicit API is not called
- topic_name: /topic_with_stamp
  sub_time:
    sec: 1646095909
    nanosec: 469289448
  sub_time_steady:
    sec: 6908353
    nanosec: 376024441
  has_header_stamp: true
  header_stamp:
    sec: 1646095909
    nanosec: 458552883
- topic_name: /topic_without_stamp
  sub_time:
    sec: 1646095909
    nanosec: 469452505
  sub_time_steady:
    sec: 6908353
    nanosec: 376187401
  has_header_stamp: false
  header_stamp:
    sec: 0
    nanosec: 0
```
