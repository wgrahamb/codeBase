cmake_minimum_required(VERSION 3.16)
project(UTIL_TESTS)
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-w)
include_directories(
	./
)
file(GLOB SOURCES "./*.cpp")
add_executable(UTIL_TESTS ${SOURCES})
