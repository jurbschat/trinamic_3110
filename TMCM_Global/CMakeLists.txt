cmake_minimum_required(VERSION 3.14)
project(TMCM_Global)

set(CMAKE_CXX_STANDARD 17)

include(../conanbuildinfo.cmake)
conan_basic_setup()

add_library(TMCM_Global STATIC TMCM_Global.cpp TMCM_GlobalClass.cpp TMCM_GlobalStateMachine.cpp)

# all other includeded cmake files will have this
target_include_directories(TMCM_Global PRIVATE ../core /usr/include/tango ${CMAKE_CURRENT_SOURCE_DIR}) # needed because of pogo generated include paths
target_link_directories(TMCM_Global PUBLIC /usr/lib/x86_64-linux-gnu/ /usr/lib/)
target_link_libraries(TMCM_Global PRIVATE tango omniDynamic4 COS4 omniORB4 omnithread log4tango zmq dl pthread stdc++ trinamic_core)