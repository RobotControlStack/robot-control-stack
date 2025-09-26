PYSRC = src
CPPSRC = src
COMPILE_MODE = Release

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
	pip install --upgrade --requirement requirements_dev.txt
	cmake -DCMAKE_BUILD_TYPE=${COMPILE_MODE} -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -B build -G Ninja
	cmake --build build --target _core

clangcompile: 
	pip install --upgrade --requirement requirements_dev.txt
	cmake -DCMAKE_BUILD_TYPE=${COMPILE_MODE} -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -B build -G Ninja
	cmake --build build --target _core

# Auto generation of CPP binding stub files
stubgen:
	pybind11-stubgen -o src --numpy-array-use-type-var rcs_robotics_library
	find ./src -name '*.pyi' -print | xargs sed -i '1s/^/# ATTENTION: auto generated from C++ code, use `make stubgen` to update!\n/'
	find ./src -not -path "./src/rcs_robotics_library/_core/*" -name '*.pyi' -delete
	find ./src/rcs_robotics_library/_core -name '*.pyi' -print | xargs sed -i 's/tuple\[typing\.Literal\[\([0-9]\+\)\], typing\.Literal\[1\]\]/tuple\[typing\.Literal[\1]\]/g'
	find ./src/rcs_robotics_library/_core -name '*.pyi' -print | xargs sed -i 's/tuple\[\([M|N]\), typing\.Literal\[1\]\]/tuple\[\1\]/g'
	ruff check --fix src/rcs_robotics_library/_core
	isort src/rcs_robotics_library/_core
	black src/rcs_robotics_library/_core



.PHONY: cppcheckformat cppformat cpplint gcccompile clangcompile stubgen
