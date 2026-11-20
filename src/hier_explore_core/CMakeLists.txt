cmake_minimum_required(VERSION 3.16)
project(hier_explore_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Pure C++17 geometry core (builds both standalone and inside colcon/ament_cmake)
add_library(geom_kernel_core STATIC
  cpp/src/geodesic_grid.cpp
  cpp/src/visibility_fov.cpp
  cpp/src/corridor_graph.cpp
)
set_target_properties(geom_kernel_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(geom_kernel_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(geom_kernel_core PRIVATE -O3 -Wall -Wextra -Wpedantic)

# Find Python3 and pybind11 (supports both apt pybind11-dev and pip pybind11)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 QUIET CONFIG)
if(NOT pybind11_FOUND)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -m pybind11 --cmakedir
    OUTPUT_VARIABLE _pybind11_cmake_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _pybind11_rc
  )
  if(_pybind11_rc EQUAL 0 AND EXISTS "${_pybind11_cmake_dir}")
    list(APPEND CMAKE_PREFIX_PATH "${_pybind11_cmake_dir}")
    find_package(pybind11 REQUIRED CONFIG)
  else()
    message(FATAL_ERROR "pybind11 not found via CMake or Python; install pybind11-dev or pip install pybind11")
  endif()
endif()

pybind11_add_module(_geom_kernel MODULE cpp/src/bindings.cpp)
target_link_libraries(_geom_kernel PRIVATE geom_kernel_core)
target_compile_options(_geom_kernel PRIVATE -O3 -Wall -Wextra)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_geom_kernel cpp/test/test_geom_kernel.cpp)
  target_link_libraries(test_geom_kernel PRIVATE geom_kernel_core)
  target_compile_options(test_geom_kernel PRIVATE -O3 -Wall -Wextra)
  add_test(NAME test_geom_kernel COMMAND test_geom_kernel)
endif()

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(ament_cmake_python REQUIRED)

  # Install Python package
  ament_python_install_package(${PROJECT_NAME})

  # Install compiled pybind11 extension alongside Python package modules
  install(
    TARGETS _geom_kernel
    LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )

  # Install ROS 2 executable entrypoint
  install(
    PROGRAMS
      scripts/coordinator_node
    DESTINATION lib/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_hier_explore_core test)
  endif()

  ament_package()
endif()
