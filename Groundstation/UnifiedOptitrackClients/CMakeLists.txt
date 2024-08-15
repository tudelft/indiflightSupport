cmake_minimum_required(VERSION 3.14)

# project and executables
project(optitrack_clients LANGUAGES CXX)

# ADD YOUR NEW CLIENT TO THIS SEMICOLON-SEPARATED LIST!
if(NOT DEFINED CLIENTS)
    message(STATUS "CLIENTS is not set. Continuing with all clients")
    set(CLIENTS "console;ivy;udp;log;ros2;ros2px4" CACHE STRING "Clients to build into the binary")
endif(NOT DEFINED CLIENTS)

add_executable(client src/main.cpp src/pose_calculations.cpp src/unified_mocap_client.cpp)

foreach(x ${CLIENTS})
    string(TOUPPER ${x} uppercase)
    add_definitions(-DUSE_CLIENT_${uppercase})
    ADD_CUSTOM_TARGET(${x}_symlink ALL
                      COMMAND ${CMAKE_COMMAND} -E create_symlink client mocap2${x}
                      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endforeach()

include_directories(include)
include_directories(clients)
include_directories(scripts)

# compiler
set (CMAKE_CXX_STANDARD 14)
if(MSVC)
  add_compile_options(/W4 /WX)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find external libraries and include for all clients
find_package(Boost REQUIRED COMPONENTS program_options filesystem)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(client ${Boost_LIBRARIES})

include(FetchContent)
fetchcontent_declare(natnet_sdk_content
  URL https://s3.amazonaws.com/naturalpoint/software/NatNetSDKLinux/ubuntu/NatNet_SDK_4.1_ubuntu.tar
)
fetchcontent_makeavailable(natnet_sdk_content)

add_library(natnet_sdk SHARED IMPORTED)
set_target_properties(natnet_sdk PROPERTIES IMPORTED_LOCATION ${natnet_sdk_content_SOURCE_DIR}/lib/libNatNet.so)
target_include_directories(natnet_sdk INTERFACE ${natnet_sdk_content_SOURCE_DIR}/include)
target_link_libraries(client natnet_sdk)

# Client-specific stuff
if ("ivy" IN_LIST CLIENTS)
    find_library(IVY_LIB NAMES ivy)
    target_link_libraries(client ivy)

    find_package(PkgConfig REQUIRED)
    pkg_search_module(GLIB REQUIRED glib-2.0)

    include_directories(${GLIB_INCLUDE_DIRS})
    link_directories(${GLIB_LIBRARY_DIRS})

    add_definitions(${GLIB_CFLAGS_OTHER})
    target_link_libraries(client ${GLIB_LIBRARIES})
endif()

if ("udp" IN_LIST CLIENTS)
    find_package(Boost REQUIRED COMPONENTS system)
endif()

if ("ros2" IN_LIST CLIENTS)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(geometry_msgs REQUIRED)

    ament_target_dependencies(client rclcpp geometry_msgs)
endif()

if ("ros2px4" IN_LIST CLIENTS)
    FetchContent_Declare(
      px4_msgs
      GIT_REPOSITORY https://github.com/PX4/px4_msgs
      GIT_TAG release/1.14
    )
    FetchContent_MakeAvailable(px4_msgs)

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(geometry_msgs REQUIRED)

    add_dependencies(client px4_msgs)
    ament_target_dependencies(client rclcpp geometry_msgs)
    target_link_libraries(client ${CMAKE_BINARY_DIR}/_deps/px4_msgs-build/libpx4_msgs__rosidl_typesupport_cpp.so)
    target_include_directories(client PUBLIC ./build/_deps/px4_msgs-build/rosidl_generator_cpp)
endif()
