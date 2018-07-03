cmake_minimum_required(VERSION 2.8.3)
project(dfExternal)
set(CMAKE_BUILD_TYPE Release)
IF(UNIX)
    set(RPC_LIB_DEFINES " -D MSGPACK_PP_VARIADICS_MSVC=0")
    set(CMAKE_CXX_FLAGS "${RPC_LIB_DEFINES} ${CMAKE_CXX_FLAGS}")    
ENDIF()

find_package(Threads REQUIRED)
set(CMAKE_CXX_FLAGS "-std=c++14 -ggdb -Wall -O3 -pthread ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "\
#                -std=c++14 -ggdb -Wall -Wextra -Wstrict-aliasing -Wunreachable-code -Wcast-qual -Wctor-dtor-privacy \
 #               -Wdisabled-optimization -Wformat=2 -Winit-self -Wmissing-include-dirs -Wswitch-default \
 #               -Wold-style-cast -Woverloaded-virtual -Wredundant-decls -Wshadow -Wstrict-overflow=5 -Wswitch-default -Wundef \
 #               -Wno-variadic-macros -Wno-parentheses -Wno-unused-function -Wno-unused -Wno-documentation -fdiagnostics-show-option \
  #              -pthread \
  #               ${CMAKE_CXX_FLAGS}")
set(BUILD_PLATFORM "x64")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)

set(GF_EXTERNAL_ROOT ${PROJECT_SOURCE_DIR})

add_subdirectory("rpclib-2.2.1")
message("***********************")
message("****  LIB  rpclib done  *****")
message("***********************")

add_subdirectory("MavLinkCom")
message("***********************")
message("****  LIB  MavLinkCom done  *****")
message("***********************")


add_subdirectory("AirLib")

