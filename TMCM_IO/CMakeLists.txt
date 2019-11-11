cmake_minimum_required(VERSION 3.13)
project(TMCM_IO)

set(CMAKE_CXX_STANDARD 14)

include(../conanbuildinfo.cmake)
conan_basic_setup()

add_library(TMCM_IO STATIC TMCM_IO.cpp TMCM_IOClass.cpp TMCM_IOStateMachine.cpp)

# all other includeded cmake files will have this
target_include_directories(TMCM_IO PRIVATE ../core /usr/include/tango ${CMAKE_CURRENT_SOURCE_DIR}) # SOURCE_DIR needed because of pogo generated include paths
target_link_directories(TMCM_IO PUBLIC /usr/lib/x86_64-linux-gnu/ /usr/lib/)
target_link_libraries(TMCM_IO PRIVATE tango omniDynamic4 COS4 omniORB4 omnithread log4tango zmq dl pthread stdc++ trinamic_core)