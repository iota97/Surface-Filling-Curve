cmake_minimum_required(VERSION 3.10...4.1)
project(surface_filling_curve)
set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE sources      src/*.cpp src/*.h)

add_executable(curve ${sources})
target_compile_options(curve PUBLIC -Wall -O3 -march=native)