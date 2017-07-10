cmake_minimum_required(VERSION 3.7)
project(NORSI_geo_2017)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp parser.h parser.cpp)
add_executable(NORSI_geo_2017 ${SOURCE_FILES})