cmake_minimum_required(VERSION 3.16)

project(PART1)

include_directories(includes)

add_executable(cmake_learn
    src/cmake_learn.cpp
    tools/hello.cpp
)