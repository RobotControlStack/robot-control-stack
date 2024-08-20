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

gcccompile: 
	pip install --upgrade --requirement requirements_dev.txt
	cmake -DCMAKE_BUILD_TYPE=${COMPILE_MODE} -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -B build -G Ninja
	cmake --build build

clangcompile: 
	pip install --upgrade --requirement requirements_dev.txt
	cmake -DCMAKE_BUILD_TYPE=${COMPILE_MODE} -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -B build -G Ninja
	cmake --build build

# Auto generation of CPP binding stub files
stubgen:
	pybind11-stubgen -o python --numpy-array-use-type-var rcsss
	find ./python -name '*.pyi' -print | xargs sed -i '1s/^/# ATTENTION: auto generated from C++ code, use `make stubgen` to update!\n/'
	find ./python -not -path "./python/rcsss/_core/*" -name '*.pyi' -delete
	find ./python/rcsss/_core -name '*.pyi' -print | xargs sed -i 's/tuple\[typing\.Literal\[\([0-9]\+\)\], typing\.Literal\[1\]\]/typing\.Literal[\1]/g'
	find ./python/rcsss/_core -name '*.pyi' -print | xargs sed -i 's/tuple\[\([M|N]\), typing\.Literal\[1\]\]/\1/g'
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
	mypy ${PYSRC} --install-types --non-interactive --no-namespace-packages

pytest:
	pytest -vv

bump:
	cz bump

commit:
	cz commit

.PHONY: cppcheckformat cppformat cpplint gcccompile clangcompile stubgen pycheckformat pyformat pylint ruff mypy pytest bump commit
