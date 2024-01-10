# Franka Interface

## Build
### Dependencies
To install the dependencies of `libfranka`` use the following command for debian based systems:
```shell
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
```
For arch based systems:
`libpoco` is `poco` and `libeigen` is `eigen` in pacman.

### Compile
```shell
cmake -B build
cmake --build build
```