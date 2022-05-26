# Tilde のビルド

ROS2 galactic 環境が構築済みのこと。

```bash
. /path/to/ros2/galactic/setup.bash

git clone git@github.com:tier4/TILDE.git

rosdep install \
  --from-paths src --ignore-src \
  --rosdistro galactic -y \
  --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

sudo apt-get install liblttng-ust-dev

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
