# tilde_cmake

This package provides CMake scripts for Tilde.

## Usage

### tilde_package.cmake

Call `tilde_package()` before defining build targets, which will set common options for Tilde.

```cmake
cmake_minimum_required(VERSION 3.5)
project(package_name)

find_package(tilde_cmake REQUIRED)
tilde_package()
find_package(tilde)
```
