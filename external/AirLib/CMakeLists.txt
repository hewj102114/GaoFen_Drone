cmake_minimum_required(VERSION 3.5.0)
project(AirLib)

# LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
# INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
# CommonSetup()

# IncludeEigen()
set(CMAKE_CXX_FLAGS "-std=c++14")    
include_directories(/home/jjt-204u/Documents/gaofen18/pro/external/eigen3)
include_directories(
  include
  ${GF_EXTERNAL_ROOT}/output/include
)

link_directories(${GF_EXTERNAL_ROOT}/output/lib)
file(GLOB_RECURSE ${PROJECT_NAME}_sources 
src/*/*.cpp

)

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_sources})

message("${CMAKE_THREAD_LIBS_INIT}")
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME} rpc)
target_link_libraries(${PROJECT_NAME} MavLinkCom)


install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include/AirLib
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN "*.in" EXCLUDE)


#string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
#find_package(Threads REQUIRED)
#target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
#target_link_libraries(${PROJECT_NAME} AirLib)
#target_link_libraries(${PROJECT_NAME} MavLinkCom)
#target_link_libraries(${PROJECT_NAME} ../AirLib/deps/rpclib/lib/${BUILD_PLATFORM}/$(Configuration)/rpc)
