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
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```
If you build with GCC >= 12 you will get a false positive array-bounds error.
Cf. [here](https://github.com/google-deepmind/mujoco/issues/1489) for solutions.
Otherwise, just use clang instead.
```shell
cmake -B build -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Release
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
Export CC and CXX env vars, if you use clang (cf. above why you might want to):
```shell`
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

The config file for the FR3 desk cli should be in yaml format:
```yaml
username=...
password=...
```
