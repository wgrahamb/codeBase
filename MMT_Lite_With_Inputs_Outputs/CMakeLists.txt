
cmake_minimum_required(VERSION 3.16)

project(MMT_Lite_With_Inputs_Outputs)

set(CMAKE_BUILD_TYPE Debug)

include_directories(include)

file(GLOB SOURCES "src/*.cpp")

add_executable(MMT_Lite ${SOURCES})
