# Tilde

see [doc](../../doc/README.md)

## About test

### test_stee_multithread.cpp

Use this with TSAN.

```
colcon build --symlink-install --packages-select tilde --mixin tsan
./build/tilde/test_stee_multithread
```

Check standard out and see TSAN message.
