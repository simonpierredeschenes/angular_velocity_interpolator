cmake_minimum_required(VERSION 3.8)
project(angular_velocity_interpolator)

add_compile_options(-march=native)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(PYTHON_EXECUTABLE /usr/bin/python3.10)

find_package (steam REQUIRED)
find_package (Eigen3 3.3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(pybind11 2.5.0 REQUIRED)
find_package (Python3 COMPONENTS Interpreter Development)

include_directories(include/angular_velocity_interpolator)
include_directories(${steam_INCLUDE_DIRS})

add_library(angular_velocity_interpolator src/angular_velocity_interpolator.cpp)
target_link_libraries(angular_velocity_interpolator ${steam_LIBRARIES} Eigen3::Eigen OpenMP::OpenMP_CXX)

add_executable(main src/main.cpp src/angular_velocity_interpolator.cpp)
target_link_libraries(main ${steam_LIBRARIES} Eigen3::Eigen OpenMP::OpenMP_CXX)

if(pybind11_FOUND)
    pybind11_add_module(pyangular_velocity_interpolator src/bindings.cpp)
    target_link_libraries(pyangular_velocity_interpolator PUBLIC angular_velocity_interpolator)
    add_dependencies(pyangular_velocity_interpolator angular_velocity_interpolator)

    if("${PYTHON_INSTALL_TARGET}" STREQUAL "")
        message(WARNING "PYTHON_INSTALL_TARGET variable is not set, the python bindings will not be installed.")
    else()
        install(TARGETS pyangular_velocity_interpolator LIBRARY DESTINATION ${PYTHON_INSTALL_TARGET})
    endif()
else()
    message(WARNING "pybind11 is not found, the python bindings will not be generated.")
endif()
