PYSRC = python
CPPSRC = src
COMPILE_MODE = Release
LINT_EXCLUDE_RUFF = --exclude examples/teleop/SimPublisher
LINT_EXCLUDE_MYPY = 'build|examples/teleop/SimPublisher|examples/inference/franka.py'

# CPP
cppcheckformat:
	clang-format --dry-run -Werror -i $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -o -name '*.h')

cppformat:
	clang-format -Werror -i $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -o -name '*.h')

cpplint: 
	clang-tidy -p=build --warnings-as-errors='*' $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -name '*.h')

# import errors
# clang-tidy -p=build --warnings-as-errors='*' $(shell find extensions/rcs_fr3/src -name '*.cpp' -o -name '*.cc' -name '*.h')

gcccompile: 
	cmake -DCMAKE_BUILD_TYPE=${COMPILE_MODE} -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -B build -G Ninja $(if ${PYTHON_EXECUTABLE},-DPython3_EXECUTABLE=${PYTHON_EXECUTABLE})
	cmake --build build --target _core

clangcompile: 
	cmake -DCMAKE_BUILD_TYPE=${COMPILE_MODE} -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -B build -G Ninja $(if ${PYTHON_EXECUTABLE},-DPython3_EXECUTABLE=${PYTHON_EXECUTABLE})
	cmake --build build --target _core

# Auto generation of CPP binding stub files
stubgen:
	pybind11-stubgen -o python --numpy-array-use-type-var --sort-by topological rcs
	find ./python -name '*.pyi' -print | xargs sed -i '1s/^/# ATTENTION: auto generated from C++ code, use `make stubgen` to update!\n/'
	find ./python -not -path "./python/rcs/_core/*" -name '*.pyi' -delete
	find ./python/rcs/_core -name '*.pyi' -print | xargs sed -i 's/tuple\[typing\.Literal\[\([0-9]\+\)\], typing\.Literal\[1\]\]/tuple\[typing\.Literal[\1]\]/g'
	find ./python/rcs/_core -name '*.pyi' -print | xargs sed -i 's/tuple\[\([M|N]\), typing\.Literal\[1\]\]/tuple\[\1\]/g'
	find ./python/rcs/_core -name '*.pyi' -print | xargs sed -i 's/class RobotConfig/class RobotConfig(typing.Generic[M])/g'
	find ./python/rcs/_core -name '*.pyi' -print | xargs sed -i 's/class SimRobotConfig(rcs._core.common.RobotConfig)/class SimRobotConfig(rcs._core.common.RobotConfig[M])/g'
	find ./python/rcs/_core -name '*.pyi' -print | xargs sed -i 's/class DynamicJointState/class DynamicJointState(typing.Generic[M])/g'
	find ./python/rcs/_core -name '*.pyi' -print | xargs sed -i 's/N = typing.TypeVar("N", bound=int)//g'
	find ./python/rcs/_core -name '*.pyi' -print | xargs sed -i 's/, N/, M/g'
	python ci_scripts/generate_common_typing.py
	ruff check --fix python/rcs/_core python/rcs/common_typing.py
	isort python/rcs/_core python/rcs/common_typing.py
	black python/rcs/_core python/rcs/common_typing.py

# Python
pycheckformat:
	isort --check-only ${PYSRC} extensions examples
	black --check ${PYSRC} extensions examples

pyformat:
	isort ${PYSRC} extensions examples
	black ${PYSRC} extensions examples

pylint: ruff mypy

ruff:
	ruff check ${PYSRC} extensions examples ${LINT_EXCLUDE_RUFF}

mypy:
	mypy ${PYSRC} extensions examples --install-types --non-interactive --no-namespace-packages --exclude ${LINT_EXCLUDE_MYPY}

pytest:
	pytest -vv

bump:
	cz bump

commit:
	cz commit

.PHONY: cppcheckformat cppformat cpplint gcccompile clangcompile stubgen pycheckformat pyformat pylint ruff mypy pytest bump commit
