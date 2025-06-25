PYSRC = python
CPPSRC = src
COMPILE_MODE = Release

# CPP
cppcheckformat:
	clang-format --dry-run -Werror -i $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -o -name '*.h')
	clang-format --dry-run -Werror -i $(shell find extensions/rcs_fr3/src -name '*.cpp' -o -name '*.cc' -o -name '*.h')

cppformat:
	clang-format -Werror -i $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -o -name '*.h')
	clang-format -Werror -i $(shell find extensions/rcs_fr3/src -name '*.cpp' -o -name '*.cc' -o -name '*.h')

cpplint: 
	clang-tidy -p=build --warnings-as-errors='*' $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -name '*.h')

# import errors
# clang-tidy -p=build --warnings-as-errors='*' $(shell find extensions/rcs_fr3/src -name '*.cpp' -o -name '*.cc' -name '*.h')

gcccompile: 
	pip install --upgrade --requirement requirements_dev.txt
	cmake -DCMAKE_BUILD_TYPE=${COMPILE_MODE} -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -B build -G Ninja
	cmake --build build --target _core

clangcompile: 
	pip install --upgrade --requirement requirements_dev.txt
	cmake -DCMAKE_BUILD_TYPE=${COMPILE_MODE} -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -B build -G Ninja
	cmake --build build --target _core

# Auto generation of CPP binding stub files
stubgen:
	pybind11-stubgen -o python --numpy-array-use-type-var rcs
	find ./python -name '*.pyi' -print | xargs sed -i '1s/^/# ATTENTION: auto generated from C++ code, use `make stubgen` to update!\n/'
	find ./python -not -path "./python/rcs/_core/*" -name '*.pyi' -delete
	find ./python/rcs/_core -name '*.pyi' -print | xargs sed -i 's/tuple\[typing\.Literal\[\([0-9]\+\)\], typing\.Literal\[1\]\]/typing\.Literal[\1]/g'
	find ./python/rcs/_core -name '*.pyi' -print | xargs sed -i 's/tuple\[\([M|N]\), typing\.Literal\[1\]\]/\1/g'
	ruff check --fix python/rcs/_core
	pybind11-stubgen -o extensions --numpy-array-use-type-var rcs_fr3
	find ./extensions/rcs_fr3 -not -path "./extensions/rcs_fr3/_core/*" -name '*.pyi' -delete
	find ./extensions/rcs_fr3 -name '*.pyi' -print | xargs sed -i '1s/^/# ATTENTION: auto generated from C++ code, use `make stubgen` to update!\n/'
	find ./extensions/rcs_fr3/_core -name '*.pyi' -print | xargs sed -i 's/tuple\[typing\.Literal\[\([0-9]\+\)\], typing\.Literal\[1\]\]/typing\.Literal[\1]/g'
	find ./extensions/rcs_fr3/_core -name '*.pyi' -print | xargs sed -i 's/tuple\[\([M|N]\), typing\.Literal\[1\]\]/\1/g'
	ruff check --fix extensions/rcs_fr3/_core
	isort python/rcs/_core extensions/rcs_fr3/_core
	black python/rcs/_core extensions/rcs_fr3/_core

# Python
pycheckformat:
	isort --check-only ${PYSRC} extensions
	black --check ${PYSRC} extensions

pyformat:
	isort ${PYSRC} extensions
	black ${PYSRC} extensions

pylint: ruff mypy

ruff:
	ruff check ${PYSRC} extensions

mypy:
	mypy ${PYSRC} extensions --install-types --non-interactive --no-namespace-packages --exclude 'build'

pytest:
	pytest -vv

bump:
	cz bump

commit:
	cz commit

.PHONY: cppcheckformat cppformat cpplint gcccompile clangcompile stubgen pycheckformat pyformat pylint ruff mypy pytest bump commit
