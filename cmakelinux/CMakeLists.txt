cmake_minimum_required(VERSION 3.22)
project(../PolyParser)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Wall")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall")

include_directories(.)

add_executable(PolyParser
        ../main.cpp
        )

add_compile_options(-O3)
