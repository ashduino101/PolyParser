cmake_minimum_required(VERSION 3.22)
project(PolyParser)

set(CMAKE_CXX_STANDARD 17)

include_directories(.)

add_executable(PolyParser
        main.cpp
        )

add_compile_options(-O3)
