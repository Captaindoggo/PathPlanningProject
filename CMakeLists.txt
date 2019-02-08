cmake_minimum_required(VERSION 2.8)
project(ASearch)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra")
set(SOURCE_FILES
        tinyxml2.cpp
        xmllogger.cpp
        isearch.cpp
        mission.cpp
        map.cpp
        dijkstra.cpp
        config.cpp
        astar.cpp
        main.cpp
        jp_search.cpp
        theta.cpp
        environmentoptions.cpp
        tinyxml2.h
        node.h
        gl_const.h
        xmllogger.h
        isearch.h
        mission.h
        map.h
        ilogger.h
        dijkstra.h
        config.h
        astar.h
        searchresult.h
        jp_search.h
        theta.h
        environmentoptions.h
        )
add_executable(ASearch ${SOURCE_FILES})



