## Samples

Simple publisher, relay and listener samples.

### pathnode::SubTimingAdvertiseNode children

#### files

- `sample_publish_header.cpp`
  - publisher sample
- `sample_relay_with_sub_timing.cpp`
  - relay sample

#### Usage

``` bash
# publisher
ros2 component standalone pathnode_sample pathnode_sample::TalkerWithHeader -r chatter:=talker

# relay
ros2 component standalone pathnode_sample pathnode_sample::RelayWithSubTiming -r in:=talker -r out:=relay1
ros2 component standalone pathnode_sample pathnode_sample::RelayWithSubTiming -r in:=relay1 -r out:=relay2

# listener - not implemented yet
```

#### check

``` bash
$ ros2 topic list
/relay1
/relay1/info/pub
/relay1/info/sub
/relay2
/relay2/info/pub
/rosout
/sample_path_info
/talker
/talker/info/pub
/talker/info/sub
```



### others

- `sample_multi_callback.cpp`
  - sample code for multiple callback group

### [BUGGY nodes] pathnode::PathNode children

- publisher node
  - `sample_publisher.cpp`
- trial and error of relay node
  - `sample_relay.cpp`
  - `sample_relay_with_path.cpp`
  - `sample_relay_with_path2.cpp`
- subscription node
  - `sample_subscriber.cpp`

