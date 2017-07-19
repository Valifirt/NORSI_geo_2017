cmake_minimum_required(VERSION 3.7)
project(NORSI_geo_2017)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES func_version/main.cpp
        func_version/parser.h
        func_version/parser.cpp
        dijkstra/dijkstra.cpp
        dijkstra/dijkstra.h)

set(CLASS_FILES
        class_version/main_for_class.cpp
        class_version/Graph.h
        class_version/Graph.cpp)

set(PARSER
        parser_osm/main_parser.cpp
        parser_osm/parser.h
        parser_osm/parser.cpp)

set(PARSER_CLASS
        parser_osm/class/main_for_class.cpp
        parser_osm/class/Graph.h
        parser_osm/class/Graph.cpp)

add_executable(func_version ${SOURCE_FILES})
add_executable(class_version ${CLASS_FILES})
add_executable(parser ${PARSER})
add_executable(parser_class ${PARSER_CLASS})
