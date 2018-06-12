cmake_minimum_required(VERSION 3.5.0)
project(MavLinkCom)

# add_subdirectory("MavLinkTest")

# LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
# INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
# CommonSetup()

include_directories(
    .
    common_utils
    include
)
set(CMAKE_CXX_FLAGS "-std=c++14")    
                
LIST(APPEND MAVLINK_SOURCES "common_utils/FileSystem.cpp")
LIST(APPEND MAVLINK_SOURCES "common_utils/ThreadUtils.cpp")
LIST(APPEND MAVLINK_SOURCES "src/MavLinkConnection.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/MavLinkFtpClient.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/MavLinkLog.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/MavLinkMessageBase.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/MavLinkMessages.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/MavLinkNode.cpp") 	
LIST(APPEND MAVLINK_SOURCES "src/MavLinkTcpServer.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/MavLinkVehicle.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/MavLinkVideoStream.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/Semaphore.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/impl/MavLinkConnectionImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/impl/MavLinkFtpClientImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/impl/MavLinkNodeImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/impl/MavLinkTcpServerImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/impl/MavLinkVehicleImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/impl/MavLinkVideoStreamImpl.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/serial_com/SerialPort.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/serial_com/TcpClientPort.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/serial_com/UdpClientPort.cpp") 
LIST(APPEND MAVLINK_SOURCES "src/serial_com/SocketInit.cpp")
LIST(APPEND MAVLINK_SOURCES "src/serial_com/wifi.cpp")

IF(UNIX)
    LIST(APPEND MAVLINK_SOURCES "src/impl/linux/MavLinkFindSerialPorts.cpp")
ELSE()
    LIST(APPEND MAVLINK_SOURCES "src/impl/windows/MavLinkFindSerialPorts.cpp")
ENDIF()

add_library(MavLinkCom STATIC ${MAVLINK_SOURCES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include/MavLinkCom
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN "*.inl"
  PATTERN "*.in" EXCLUDE)
