# Franka Interface

## Build
### Dependencies
To install the dependencies of `libfranka` use the following command for debian based systems:
```shell
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev libxslt-dev libcoin-dev libccd-dev libglfw3-dev clang libboost-all-dev liblzma-dev
```
For arch based systems:
`libpoco` is `poco` and `libeigen` is `eigen` in pacman.

### Compile
```shell
cmake -B buila -G Ninja
cmake --build build
```

### Python Bindings
```shell
# create new virtual env and activate it

# install pyfr3
pip install .

# add dynamic linking paths
export LD_LIBRARY_PATH=build/lib:build/_deps/rl-build/lib
```
Open python and try to import the lib:
```python
import pyfr3
```