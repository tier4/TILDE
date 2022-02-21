## classes

| name                               | about                                                                |
|------------------------------------|----------------------------------------------------------------------|
| pathnode::PathNode                 | BUGGY. Node with custom `create_subscription`. **Should be deleted** |
| pathnode::TildeNode                | Successor of PathNode which also has custom `create_publisher`.      |
| pathnode::TimingAdvertisePublisher | custom publisher                                                     |

## TimingAdvertisePublisher



## history

- 2021/11
  - enrich publisher topic info entries to know the input information generation time
- 2021/9, 2021/10
  - Poc of topic info
  - send topic info at the beginning of subscription callback and at the timing of publishing


