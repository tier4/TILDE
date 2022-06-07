# tilde_vis_test

## Description

Supplementary files to test `tilde_vis` package.

## issues_23_1.launch.py

See #23. It's OK if the program runs with no error.

``` bash
$ ros2 launch tilde_vis_test issues_23_1.launch.py
[filter-1] [INFO] [1654561227.800326321] [filter]: Filter get message cnt_ = 0
[filter-1] [INFO] [1654561228.300162503] [filter]: Filter get message cnt_ = 1
[filter-1] [INFO] [1654561228.800174003] [filter]: Filter get message cnt_ = 2
[filter-1] [INFO] [1654561229.300167408] [filter]: Filter get message cnt_ = 3
[filter-1] [INFO] [1654561229.800184144] [filter]: Filter get message cnt_ = 4
```

## issues_23_2.launch.py

See #23. It's OK if the program runs with no error.
Be aware that FPS is very slow to reproduce bug.

``` bash
$ ros2 launch tilde_vis_test issues_23_2.launch.py

[relay-1] [INFO] [1654561285.390170828] [relay]: Relay get message cnt_ = 0
[relay-1] [INFO] [1654561290.390119277] [relay]: Relay get message cnt_ = 1
[relay-1] [INFO] [1654561295.390336942] [relay]: Relay get message cnt_ = 2
```
