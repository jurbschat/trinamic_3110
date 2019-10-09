cmake_minimum_required(VERSION 3.13)
project(trinamic_3110)

set(CMAKE_CXX_STANDARD 14)

include(conanbuildinfo.cmake)
conan_basic_setup()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic")

add_subdirectory(core)
add_subdirectory(TMCM_Motor)
add_subdirectory(TMCM_Global)

add_executable(trinamic_3110 main.cpp MultiClassesFactory.cpp tangohelpers.h tangohelpers.cpp)
target_include_directories(trinamic_3110 SYSTEM PRIVATE ../core /usr/include/tango ${CMAKE_CURRENT_SOURCE_DIR} TMCM_Global TMCM_Motor) # needed because of pogo generated include paths
target_link_libraries(trinamic_3110 PRIVATE tango omniDynamic4 COS4 omniORB4 omnithread log4tango zmq dl pthread stdc++ TMCM_Motor TMCM_Global)
target_link_directories(trinamic_3110 PUBLIC /usr/lib/x86_64-linux-gnu/ /usr/lib/)