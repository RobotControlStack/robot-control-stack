# Robot Control Stack

## Build
### Dependencies
To install the dependencies of `libfranka` use the following command for debian based systems:
```shell
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev libxslt-dev libcoin-dev libccd-dev libglfw3-dev clang libboost-all-dev liblzma-dev ninja-build clang-format clang clang-tidy
```
For arch based systems:
`libpoco` is `poco` and `libeigen` is `eigen` in pacman.

For wrlview program qt must be installed
```shell
sudo apt-get install qt5-qmake qtbase5-dev libsoqt520-dev
```

### Compile
```shell
cmake -B build -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### Formatting and Linting
```shell
# check for formatting errors
clang-format --dry-run -Werror -i $(find src -name '*.cpp' -o -name '*.cc' -o -name '*.h')
# fix them
clang-format <file_name>
# Linting
clang-tidy -p=build --warnings-as-errors='*' $(find src -name '*.cpp' -o -name '*.cc' -name '*.h')
```

### Python Bindings
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