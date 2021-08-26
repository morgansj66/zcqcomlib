cmake_minimum_required(VERSION 3.11)
project(listChannels C)
project(canmonitor C)
project(canfdmonitor C)
project(6chrxtest CXX)

# set the C++14 standard
set(CMAKE_CXX_STANDARD 14)
if (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif ()

# I../includes
include_directories(include)

add_executable (listChannels listChannels.c)
target_link_libraries(listChannels zcqcomlib)
# set_target_properties(listChannels PROPERTIES LINK_FLAGS /SUBSYSTEM:WINDOWS)

add_executable (canmonitor canmonitor.c)
target_link_libraries(canmonitor zcqcomlib)

add_executable (canfdmonitor canfdmonitor.c)
target_link_libraries(canfdmonitor zcqcomlib)

add_executable (6chrxtest 6chrxtest.cpp)
target_link_libraries(6chrxtest zcqcomlib)
