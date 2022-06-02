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

- Manually copy & rename corresponding files
- Prepare TildeSubscriber
  - add `#include "tilde_message_filters/tilde_subscriber.hpp"`
  - add `using namespace tilde_message_filters;`
  - replace `Subscriber` to `TildeSubscriber`
- Use `tilde::TildeNode` instead of `rclcpp::Node`
  - add `#include "tilde/tilde_node.hpp"`
  - replace `rclcpp::Node` to `tilde::TildeNode`
- Comment tests using LifecycleNode
  - `TEST(TildeSubscriber, lifecycle)` on 20220421

#### test_synchronizer.cpp

- Manually copy & rename corresponding files
- Prepare TildeSynchronizer

  - add `#include "tilde_message_filters/tilde_synchronizer.hpp"`
  - add `using namespace tilde_message_filters;`
  - replace `Synchronizer` to `TildeSynchronizer` of TEST
    - don't change `typedef Synchronizer<NullPolicy> Sync;` in `struct NullPolicy`
  - Add `TildeNode` to TildeSynchronizer construstors
    - `nullptr` may work
    - non-null instance is needed for message handling tests.
      (add2, add3, and so on)
      If null, the results are unexpectable.

- Use `tilde::TildeNode` instead of `rclcpp::Node`
  - add `#include "tilde/tilde_node.hpp"`
  - replace `rclcpp::Node` to `tilde::TildeNode`
- Comment tests using LifecycleNode
  - `TEST(TildeSubscriber, lifecycle)` on 20220421
