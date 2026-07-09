add_library(bilateral_hw)
target_sources(bilateral_hw PRIVATE BilateralFranka.cpp)
target_include_directories(
  bilateral_hw
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${RCS_FR3_EXTENSION_DIR}/src
)
target_link_libraries(
  bilateral_hw
  PUBLIC hw rcs pinocchio::all Eigen3::Eigen
)
