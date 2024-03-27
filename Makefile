PYSRC = python/rcsss
CPPSRC = src

# CPP
cppcheckformat:
	clang-format --dry-run -Werror -i $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -o -name '*.h')

.PHONY: cppformat
cppformat:
	clang-format -Werror -i $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -o -name '*.h')

.PHONY: cpplint
cpplint: 
	clang-tidy -p=build --warnings-as-errors='*' $(shell find ${CPPSRC} -name '*.cpp' -o -name '*.cc' -name '*.h')

.PHONY: compile
compile: 
	cmake -DCMAKE_BUILD_TYPE=Release -B build -G Ninja
	cmake --build build

.PHONY: clangcompile
compile: 
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -B build -G Ninja -S .
	cmake --build build

# Auto generation of CPP binding stub files
.PHONY: genstub
genstub:
	pybind11-stubgen -o python --numpy-array-use-type-var rcsss
	find ./python -name '*.pyi' -print | xargs sed -i '1s/^/# ATTENTION: auto generated from C++ code, use `make genstub` to update!\n/'
	find ./python -not -path "./python/rcsss/_core/*" -name '*.pyi' -delete
	isort python/rcsss/_core/*.pyi
	black python/rcsss/_core/*.pyi

# Python
.PHONY: pycheckformat
pycheckformat:
	isort --check-only ${PYSRC}
	black --check ${PYSRC}

.PHONY: pyformat
pyformat:
	isort ${PYSRC}
	black ${PYSRC}

.PHONY: pylint
pylint: ruff mypy

.PHONY: ruff
ruff:
	ruff check ${PYSRC}

.PHONY: mypy
mypy:
	mypy ${PYSRC} --install-types --non-interactive
