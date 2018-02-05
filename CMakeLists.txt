cmake_minimum_required(VERSION 2.8)
project(AStar-JPS-ThetaStar)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra")
set(SOURCE_FILES
        asearch.cpp
        bfs.cpp
        config.h
        environmentoptions.cpp
        ilogger.h
        jp_search.cpp
        mission.cpp
        searchresult.h
        xmllogger.cpp
        astar.cpp
        bfs.h
        dijkstra.cpp
        environmentoptions.h
        isearch.cpp
        jp_search.h
        map.cpp
        mission.h
        theta.cpp
        xmllogger.h
        astar.h
        config.cpp
        dijkstra.h
        gl_const.h
        isearch.h
        map.h
        node.h
        theta.h
        tinyxml2.cpp
        tinyxml2.h
        )
add_executable(AStar-JPS-ThetaStar ${SOURCE_FILES})