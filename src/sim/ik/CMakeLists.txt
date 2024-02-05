add_library(ik STATIC ik.cpp ik.h)
target_link_libraries(ik PRIVATE hal mdl util)
