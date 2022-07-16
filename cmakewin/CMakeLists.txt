cmake_minimum_required(VERSION 3.22)
project(../PolyParser)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_SYSTEM_NAME Windows)

SET(CMAKE_C_COMPILER i686-w64-mingw32-cc)
SET(CMAKE_CXX_COMPILER i686-w64-mingw32-c++)
SET(CMAKE_RC_COMPILER i686-w64-mingw32-windres)
set(CMAKE_RANLIB i686-w64-mingw32-ranlib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS)

set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Wall")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall")

include_directories(.)

add_executable(PolyParser
        ../main.cpp
        )

add_compile_options(-O3)
