cmake_minimum_required (VERSION 3.18)
project(optitrack_forwarder)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD C++11)

#target_link_libraries(connect ${PROJECT_NAME})

# ----- python generation of pi protocol headers -----
# find Py3, setting the Python3_EXECUTATBLE CMAKE variable
find_package(Python3 COMPONENTS Interpreter)
#message(${Python3_EXECUTABLE})

set(NATNET_SRC_DIR ${CMAKE_SOURCE_DIR}/ext/natnet3x)

configure_file(${CMAKE_SOURCE_DIR}/src/natnet2udp.py ${CMAKE_CURRENT_BINARY_DIR}/natnet2udp.py COPYONLY)
configure_file(${NATNET_SRC_DIR}/NatNetClient.py ${CMAKE_CURRENT_BINARY_DIR}/natnet3x/NatNetClient.py COPYONLY)
