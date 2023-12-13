# TILDE - Tilde Is Latency Data Embedding

[README(ja)](./doc/README.md)

## Publications & presentations

If you find TILDE is useful in your research, please consider citing:

- Xuankeng He, Hiromi Sato, Yoshikazu Okumura, and Takuya Azumi, "TILDE: Topic-tracking Infrastructure for Dynamic Message Latency and Deadline Evaluator for ROS 2 Application," The 27th International Symposium on Distributed Simulation and Real Time Applications (DS-RT), 2023.

<details>
<summary>BibTeX</summary>

```bibtex
@inproceedings{TILDE,
title={{TILDE}: {Topic-tracking} {Infrastructure} for {Dynamic} {Message Latency} and {Deadline Evaluator} for {ROS} 2 {Application}},
author={Xuankeng He, Hiromi Sato, Yoshikazu Okumura, and Takuya Azumi},
booktitle={Proceedings of The 27th International Symposium on Distributed Simulation and Real Time Applications (DS-RT)},
year={2023}}
```
</details>

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
