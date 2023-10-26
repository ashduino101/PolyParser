cmake_minimum_required(VERSION 3.21)
project(PolyParser)

set(CMAKE_CXX_STANDARD 20)

include_directories(.)

add_executable(PolyParser
        main.cpp
        )

add_compile_options(-O3)
