PYSRC = python/rcsss
CPPSRC = src

# CPP
.PHONY: cppcheckformat
cppcheckformat:
	clang-format --dry-run -Werror -i $(find $(CPPSRC) -name '*.cpp' -o -name '*.cc' -o -name '*.h')

.PHONY: cppformat
cppformat:
	clang-format -Werror -i $(find $(CPPSRC) -name '*.cpp' -o -name '*.cc' -o -name '*.h')

.PHONY: cpplint
cpplint: 
	clang-tidy -p=build --warnings-as-errors='*' $(find $(CPPSRC) -name '*.cpp' -o -name '*.cc' -name '*.h')

.PHONY: compile
compile: 
	cmake -DCMAKE_BUILD_TYPE=Release -B build -G Ninja
	cmake --build build

.PHONY: clangcompile
compile: 
	cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -B build -G Ninja -S .
	cmake --build build

# Python
.PHONY: pycheckformat
pycheckformat:
	isort --check-only $(PYSRC)
	black --check $(PYSRC)

.PHONY: pyformat
pyformat:
	isort $(PYSRC)
	black $(PYSRC)

.PHONY: pylint
pylint: ruff mypy

.PHONY: ruff
ruff:
	ruff check $(PYSRC)

.PHONY: mypy
mypy:
	mypy $(PYSRC) --explicit-package-bases