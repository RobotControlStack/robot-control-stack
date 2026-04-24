PYSRC = python
CPPSRC = src
COMPILE_MODE = Release
LINT_EXCLUDE_RUFF = --exclude examples/teleop/SimPublisher
LINT_EXCLUDE_MYPY = 'build|examples/teleop/SimPublisher'

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
	sed -i 's/    q_home: numpy\.ndarray\[tuple\[M\], numpy\.dtype\[numpy\.float64\]\] | None/    q_home: numpy.ndarray | None/' python/rcs/_core/common.pyi
	python -c "from pathlib import Path; p=Path('python/rcs/_core/common.pyi'); t=p.read_text(); t=t.replace('numpy.ndarray[tuple[typing.Literal[2], N], numpy.dtype[numpy.float64]]', 'numpy.ndarray[tuple[typing.Literal[2], typing.Any], numpy.dtype[numpy.float64]]'); p.write_text(t)"
	python -c "from pathlib import Path; p=Path('python/rcs/_core/sim.pyi'); t=p.read_text(); t=t.replace('numpy.ndarray[tuple[typing.Literal[2], N], numpy.dtype[numpy.float64]]', 'numpy.ndarray[tuple[typing.Literal[2], typing.Any], numpy.dtype[numpy.float64]]'); t=t.replace(', max_buffer_frames: int = 100', ''); p.write_text(t)"
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
