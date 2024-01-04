# Franka Interface

## Requirements

- Class that can control n robots 
  - config file with ips
  - collision checker
  - simple cartesian position and joint position move command
  - python wrapper with pybind
- hardware abstraction layer with robotics lib

## Build

### Dependencies

To install the dependencies of libfranka use the following command for debian
based systems:

`sudo apt install build-essential cmake git libpoco-dev libeigen3-dev`

(libpoco is `poco` and libeigen is `eigen` in pacman)

### Compile
```shell
# cmake setup
cmake -B build
# cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -B build
# cmake -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -S . -B build -G Ninja

# to build
cmake --build build
```