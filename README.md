PathNode sample
===

## Description

PoC of deadline detection framework

## Contents

| src/            | about                                                |
|-----------------|------------------------------------------------------|
| tilde           | deadline detection framework                         |
| pathnode_sample | samples including talker, relay, listener and so on. |
| tilde_msg       | FW internal msg                                      |

## Build

```
. /path/to/ros2_galactic/install/setup.bash
colcon build --symlink-install
```

## Run samples

The pathnode_samples are implemented as ROS2 components.

### simple relay

Typical sample is the topic flow such as `/talker -> /relay1 -> /relay2 -> /listener`.

```
# term1: talker talks to `/talker`
ros2 component standalone pathnode_sample pathnode_sample::Talker -r chatter:=talker

# term2: relay `/talker` -> `/relay1`
ros2 component standalone pathnode_sample pathnode_sample::Relay -r in:=talker -r out:=relay1
  # (no output)

# term3: relay: `relay1` -> `relay2`
ros2 component standalone pathnode_sample pathnode_sample::Relay -r in:=relay1 -r out:=relay2
  # (no output)

# term4: relay: `relay2` -> `listener`
ros2 component standalone pathnode_sample pathnode_sample::Relay -r in:=relay2 -r out:=listener
  # (no output)

# term5: listener subscribes `/listener`
ros2 component standalone pathnode_sample pathnode_sample::Listener -r chatter:=listener
  # [INFO] [1631063086.571361319] [listener]: I heard: [Hello World: 32]
  # [INFO] [1631063087.571401747] [listener]: I heard: [Hello World: 33]
```

## relay with path deadline

RelayWithPath can define path with deadline.

Now, we have the following graph:

```
RN means "relay_node".

(Talker) -> /talker -(RN1)-> /relay1 -(RN2)-> /relay2 -(RN3)-> /listener -> (Listener)

                    |---------------------------------------|
                                 sample_path

Now, each RN has the path deadline.
Path starts at RN1 "/talker" subscription callback.
The subscription callbasks of "/relay1" and `/relay2` should end by deadline.
We can define the start and end timing of each callback by "valid_min" and "valid_max".
```

In the following commands, we set:
- relay_node1 `/talker` callback should be done when t is in [0, 2] [ms]
- relay_node2 `/relay1` callback should be done when t is in [0, 4] [ms]
- relay_node2 `/relay2` callback should be done when t is in [0, 6] [ms]

Now t=0 means the timing when the path starts.
Optionally, each callback can sleep for `wait_msec` [ms].

```
# talker
ros2 component standalone pathnode_sample pathnode_sample::Talker -r chatter:=talker

# relay_node1
ros2 component standalone pathnode_sample pathnode_sample::RelayWithPath \
   -r __node:=relay_node1 \
   -r in:=talker -r out:=relay1 \
   -p is_first:=true \
   -p path_valid_min_sec:=0 \
   -p path_valid_min_ns:=0 \
   -p path_valid_max_sec:=0 \
   -p path_valid_max_ns:=2000000 \
   -p wait_msec:=0


# relay_node22
ros2 component standalone pathnode_sample pathnode_sample::RelayWithPath \
   -r __node:=relay_node2 \
   -r in:=relay1 -r out:=relay2 \
   -p is_first:=false \
   -p path_valid_min_sec:=0 \
   -p path_valid_min_ns:=0 \
   -p path_valid_max_sec:=0 \
   -p path_valid_max_ns:=4000000 \
   -p wait_msec:=0

# relay_node3
ros2 component standalone pathnode_sample pathnode_sample::RelayWithPath \
   -r __node:=relay_node3 \
   -r in:=relay2 -r out:=listener \
   -p is_first:=false \
   -p path_valid_min_sec:=0 \
   -p path_valid_min_ns:=0 \
   -p path_valid_max_sec:=0 \
   -p path_valid_max_ns:=6000000 \
   -p wait_msec:=0

# listener
ros2 component standalone pathnode_sample pathnode_sample::Listener -r chatter:=listener
```

## Test of long callbacks

To check executor behaviors, we can use `pathnode_sample::SampleMultiCallback`.

It has the following callbacks:
- three subscription callbacks `sub1` - `sub3`
  - subscribes `sub1`
  - in callback,
    - sleeps for `sub1_wait_ms` [ms] (default 0 ms)
    - finally publishes to `pub1`.
  - same for `sub2` and `sub3`
- two timer callbacks: `timer1` and `timer2`
  - wake up periodically every `timer1_interval_ms` [ms] (default 1000 ms)
  - in callback
    - sleeps for `timer1_wait_ms` [ms] (default 0 ms)
    - finally publishes to `timer_pub1`
  - same for `timer2`

In default, all subscription callbacks and timers wake up every 1[s] and send message without sleep.

```
# talker
ros2 component standalone pathnode_sample pathnode_sample::Talker -r chatter:=sub1
ros2 component standalone pathnode_sample pathnode_sample::Talker -r chatter:=sub2
ros2 component standalone pathnode_sample pathnode_sample::Talker -r chatter:=sub3

# sample_multi_callback
ros2 run rclcpp_components component_container_mt

ros2 component load /ComponentManager pathnode_sample pathnode_sample::SampleMultiCallback \
  -p sub1_wait_ms:=5000

# If you want check each callback pub, do like these
ros2 topic echo /pub2
ros2 topic echo /timer_pub1
```



