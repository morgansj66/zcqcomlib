cmake_minimum_required(VERSION 3.11)
project(listChannels C)
project(canmonitor C)
project(canfdmonitor C)

# set the C++14 standard
set(CMAKE_CXX_STANDARD 14)

# I../includes
include_directories(include)
add_executable (listChannels listChannels.c)
target_link_libraries(listChannels zcqcomlib)
add_executable (canmonitor canmonitor.c)
target_link_libraries(canmonitor zcqcomlib)
add_executable (canfdmonitor canfdmonitor.c)
target_link_libraries(canfdmonitor zcqcomlib)

