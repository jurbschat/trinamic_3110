cmake_minimum_required(VERSION 3.13)
project(TMCM_Motor)

set(CMAKE_CXX_STANDARD 14)

include(../conanbuildinfo.cmake)
conan_basic_setup()

add_library(TMCM_Motor STATIC TMCM_Motor.cpp  TMCM_MotorClass.cpp TMCM_MotorStateMachine.cpp)

# all other includeded cmake files will have this
target_include_directories(TMCM_Motor PRIVATE ../core /usr/include/tango ${CMAKE_CURRENT_SOURCE_DIR}) # needed because of pogo generated include paths
target_link_directories(TMCM_Motor PUBLIC /usr/lib/x86_64-linux-gnu/ /usr/lib/)
target_link_libraries(TMCM_Motor PRIVATE tango omniDynamic4 COS4 omniORB4 omnithread log4tango zmq dl pthread stdc++ trinamic_core)