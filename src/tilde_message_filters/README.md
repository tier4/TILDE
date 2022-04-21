

## TODO

- [ ] consider pointer from TildeSynchronizer or TildeSubscriber to TildeNode
  - originally, it is not required
  - shared_ptr could be cyclic pointers
  - now raw pointer is used (weak_pointer may be a good replacement)

## UT

### compatibility test

To check compatibility, we use the following tests:

- test_from_original_subscriber.cpp
- test_from_original_synchronizer.cpp

See bellow to know how to maintenance.

#### test_subscriber.cpp

- Mnually copy & rename corresponding files
- Prepare TildeSubscriber
  - replace `#include "message_filters/subscriber.h"` to
    `#include "tilde_message_filters/tilde_subscriber.h"`
  - add `using namespace tilde_message_filters;`
  - replace `Subscriber` to `TildeSubscriber`
- Use `tilde::TildeNode` instead of `rclcpp::Node`
  - add `#include "tilde/tilde_node.hpp"`
  - replace `rclcpp::Node` to `tilde::TildeNode`
- Comment tests using LifecycleNode
  - `TEST(TildeSubscriber, lifecycle)` on 20220421

