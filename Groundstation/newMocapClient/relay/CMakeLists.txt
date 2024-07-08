cmake_minimum_required (VERSION 3.18)
project(relay)

set(PI_GEN_DIR ${CMAKE_BINARY_DIR}/pi-protocol)
include_directories(${PI_GEN_DIR})

# compile with pi-stats support and message printing
add_compile_options(-Wall -Wpedantic -Werror) #todo: find the proper way to do this
add_definitions(-DPI_STATS -DPI_USE_PRINT_MSGS -DPI_DEBUG)

add_executable(relay ${CMAKE_SOURCE_DIR}/src/relay.cpp
    ${PI_GEN_DIR}/pi-protocol.c
    ${PI_GEN_DIR}/pi-messages.c
)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD C++11)

#target_link_libraries(connect ${PROJECT_NAME})

# ----- python generation of pi protocol headers -----
# find Py3, setting the Python3_EXECUTATBLE CMAKE variable
find_package(Python3 COMPONENTS Interpreter)
#message(${Python3_EXECUTABLE})

set(PI_SRC_DIR ${CMAKE_SOURCE_DIR}/ext/pi-protocol)
set(PI_CONFIG_FILENAME config.yaml)
add_custom_command(
    OUTPUT
        ${PI_GEN_DIR}/pi-protocol.h
        ${PI_GEN_DIR}/pi-messages.h
        ${PI_GEN_DIR}/pi-messages.c
    DEPENDS
        ${PI_SRC_DIR}/${PI_CONFIG_FILENAME}
        ${PI_SRC_DIR}/msgs/*.yaml
        ${PI_SRC_DIR}/python/*.py
        ${PI_SRC_DIR}/templates/*.j2
    COMMAND mkdir -p ${PI_GEN_DIR} && ${Python3_EXECUTABLE}
        ${PI_SRC_DIR}/python/generate.py
        ${PI_CONFIG_FILENAME}
        --output-dir ${PI_GEN_DIR})
configure_file(${PI_SRC_DIR}/src/pi-protocol.c ${PI_GEN_DIR}/pi-protocol.c COPYONLY)

# startup file, not used anymore in favour of the start script in the root of racing_pi
#configure_file(${CMAKE_SOURCE_DIR}/src/start.sh ${CMAKE_CURRENT_BINARY_DIR}/start.sh COPYONLY)

# unit file to keep client started at all times, see README
configure_file(${CMAKE_SOURCE_DIR}/src/relay.service ${CMAKE_CURRENT_BINARY_DIR}/relay.service COPYONLY)
