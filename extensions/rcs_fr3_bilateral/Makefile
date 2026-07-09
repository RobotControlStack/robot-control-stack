PYSRC = src/rcs_fr3_bilateral
CPPSRC = src
COMPILE_MODE = Release

cppcheckformat:
	clang-format --dry-run -Werror -i $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -o -name '*.h')

cppformat:
	clang-format -Werror -i $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -o -name '*.h')

cpplint:
	clang-tidy -p=build --warnings-as-errors='*' $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -name '*.h')

gcccompile:
	cmake -DCMAKE_BUILD_TYPE=${COMPILE_MODE} -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -B build -G Ninja
	cmake --build build --target _core

clangcompile:
	cmake -DCMAKE_BUILD_TYPE=${COMPILE_MODE} -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -B build -G Ninja
	cmake --build build --target _core

stubgen:
	pybind11-stubgen -o src --numpy-array-use-type-var --sort-by topological rcs_fr3_bilateral
	find ./${PYSRC} -name '*.pyi' -print | xargs sed -i '1s/^/# ATTENTION: auto generated from C++ code, use `make stubgen` to update!\n/'
	find ./${PYSRC} -not -path "./${PYSRC}/_core/*" -name '*.pyi' -delete
	ruff check --fix ${PYSRC}/_core
	isort ${PYSRC}/_core
	black ${PYSRC}/_core

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
	mypy ${PYSRC} --install-types --non-interactive --no-namespace-packages --exclude 'build'

.PHONY: cppcheckformat cppformat cpplint gcccompile clangcompile stubgen pycheckformat pyformat pylint ruff mypy
