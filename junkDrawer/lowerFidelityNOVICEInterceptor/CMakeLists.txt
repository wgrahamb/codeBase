cmake_minimum_required (VERSION 2.8.11)
project (interceptor)
set(CMAKE_BUILD_TYPE Debug)
add_executable(interceptor interceptor.cpp util.cpp)