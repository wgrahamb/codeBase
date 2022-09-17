cmake_minimum_required(VERSION 3.16)
project(70MM_ROCKET)
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-w)
include_directories(
	./
	./Utility
)
file(GLOB SOURCES "./*.cpp" "./Utility/*.cpp")
add_executable(70MM_ROCKET ${SOURCES})
