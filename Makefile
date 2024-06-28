PYSRC = python/rcsss
CPPSRC = src
COMPILE_MODE = Release

# CPP
cppcheckformat:
	clang-format --dry-run -Werror -i $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -o -name '*.h')

cppformat:
	clang-format -Werror -i $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -o -name '*.h')

cpplint: 
	clang-tidy -p=build --warnings-as-errors='*' $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -name '*.h')

# NOTE: when changing the version, also change it in the pyproject.toml
MUJOCO_VERSION=3.1.5
gcccompile: 
	cmake -DCMAKE_BUILD_TYPE=${COMPILE_MODE} -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -DMUJOCO_VERSION=${MUJOCO_VERSION} -B build -G Ninja
	cmake --build build

clangcompile: 
	cmake -DCMAKE_BUILD_TYPE=${COMPILE_MODE} -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DMUJOCO_VERSION=${MUJOCO_VERSION} -B build -G Ninja
	cmake --build build

# Auto generation of CPP binding stub files
stubgen:
	pybind11-stubgen -o python --numpy-array-use-type-var rcsss
	find ./python -name '*.pyi' -print | xargs sed -i '1s/^/# ATTENTION: auto generated from C++ code, use `make stubgen` to update!\n/'
	find ./python -not -path "./python/rcsss/_core/*" -name '*.pyi' -delete
	find ./python/rcsss/_core -name '*.pyi' -print | xargs sed -i 's/tuple\[typing\.Literal\[\([0-9]\+\)\], typing\.Literal\[1\]\]/typing\.Literal[\1]/g'

	ruff check --fix python/rcsss/_core
	isort python/rcsss/_core
	black python/rcsss/_core

# Python
pycheckformat:
	isort --check-only ${PYSRC}
	black --check ${PYSRC}

pyformat:
	isort ${PYSRC}
	black ${PYSRC}

pylint: ruff mypy

ruff:
	ruff check ${PYSRC}

mypy:
	mypy ${PYSRC} --install-types --non-interactive

pytest:
	pytest -vv

.PHONY: cppcheckformat cppformat cpplint gcccompile clangcompile stubgen pycheckformat pyformat pylint ruff mypy pytest
