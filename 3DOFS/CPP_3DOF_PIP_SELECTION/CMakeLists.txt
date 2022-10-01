cmake_minimum_required (VERSION 2.8.11)
project (missileModel)
set(CMAKE_BUILD_TYPE Debug)
include_directories(
	./
)
file(GLOB SOURCES "./*.cpp")
add_executable(missileModel ${SOURCES})