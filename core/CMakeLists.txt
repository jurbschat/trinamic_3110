cmake_minimum_required(VERSION 3.13)
project(trinamic_core)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic")

include(../conanbuildinfo.cmake)
conan_basic_setup()

add_library(trinamic_core core.cpp)
#target_include_directories(trinamic_core SYSTEM PRIVATE)
target_link_libraries(trinamic_core PRIVATE boost_system pthread ${CONAN_LIBS})