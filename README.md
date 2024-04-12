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
make gcccompile
```
If you build with GCC >= 12 you will get a false positive array-bounds error.
Cf. [here](https://github.com/google-deepmind/mujoco/issues/1489) for solutions.
Otherwise, just use clang instead.
```shell
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
```
Export CC and CXX env vars, if you use clang (cf. above why you might want to):
```shell
export CC=/usr/bin/clang
export CXX=/usr/bin/clang++
```

then install the pip package:
```shell
pip install .
```

then stubgen the genstub bindings
```shell
make stubgen
```

Import the library in python:
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

## Hardware
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
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:venv/lib/python3.xy/site-packages/k4a/_libs
```
### Franka Desk
The config file for the FR3 desk cli should be in yaml format:
```yaml
username=...
password=...
```
