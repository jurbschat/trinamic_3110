cmake_minimum_required(VERSION 3.14)
project(trinamic_3110_tango)

set(CMAKE_CXX_STANDARD 17)

add_executable(trinamic_3110_tango main.cpp)
target_include_directories(trinamic_3110_tango SYSTEM PRIVATE libs/boost_1_71_0/)
target_link_libraries(trinamic_3110_tango PRIVATE boost_system pthread)
target_link_directories(trinamic_3110_tango PRIVATE libs/boost_1_71_0/stage/lib)