cmake_minimum_required (VERSION 3.10)

# Define the project
project(timing_test)

# GoogleTest requires at least C++11
set(CMAKE_CXX_STANDARD 11)


# Add definition for the simple-c-setup binary
add_executable(timing_test non_pcl.cpp)