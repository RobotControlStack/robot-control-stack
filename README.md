# Robot Control Stack

## C++ build
### Dependencies
To install the dependencies use the following command for debian based systems:
```shell
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev libxslt-dev libcoin-dev libccd-dev libglfw3-dev clang libboost-all-dev liblzma-dev ninja-build clang-format clang clang-tidy
```
For arch based systems:
`libpoco` is `poco` and `libeigen` is `eigen` in pacman.

In order to use the `wrlview` program from the `rl` library, you have to install QT. On debian-based system use:
```shell
sudo apt-get install qt5-qmake qtbase5-dev libsoqt520-dev
```

### Compile
```shell
# default compiler
make compile
# clang compiler
make clangcompile
```

### Formatting and Linting
```shell
# check for formatting errors
make cppcheckformat
# fix them
make cppformat
# Linting with clang tidy
make cpplint
```

## Python Bindings
```shell
# create new virtual env and activate it
virtualenv --python=python3.11 venv
source venv/bin/activate

# export CC and CXX env vars to use clang compile (needed for mujoco)
export CC=/usr/bin/clang
export CXX=/usr/bin/clang++

# install rcsss
pip install .

# add dynamic linking paths
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:build/lib:build/_deps/rl-build/lib
```
Open python and try to import the lib:
```python
import rcsss
```
### Formatting and Linting
```shell
# before you need to install the linter and formatter dependencies:
pip install -r requirements_dev.txt
# check for formatting errors
make pycheckformat
# fix them
make pyformat
# Linting with ruff and mypy
make pylint
```

## Sensors
### Microsoft Azure Kinect
Use the following install script to install k4a on debian-based systems:
```shell
# source your virtual env before
source venv/bin/activate
# install the python lib
./install_kinect_4ak.sh
```
or download the wheel package from the github pipeline artifacts and install it with
```shell
# source your virtual env before
source venv/bin/activate
# install wheel
python -m pip install <name>.whl
```

In order to avoid putting `libdepthengine.so.2.0` into `/usr/lib/x86_64-linux-gnu` (see [this issue](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1707)) export `LD_LIBRARY_PATH`:
```shell
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:venv/lib/python3.11/site-packages/k4a/_libs
```