cmake_minimum_required(VERSION 2.8)

project(lyncs_rover CXX)

set (LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/py_src/bin)

include_directories(src)
add_subdirectory(src)
