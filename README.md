# TILDE - Tilde Is Latency Data Embedding

[README(ja)](./doc/README.md)

## Contents

| src/         | about                                                |
| ------------ | ---------------------------------------------------- |
| tilde        | TILDE main code                                      |
| tilde_sample | samples including talker, relay, listener and so on. |
| tilde_msg    | TILDE internal msg                                   |
| tilde_vis    | TILDE results visualization                          |

## Build

```bash
. /path/to/ros2_galactic/install/setup.bash
vcs import src < build_depends.repos
colcon build --symlink-install --cmake-args --cmake-args -DCMAKE_BUILD_TYPE=Release
```
