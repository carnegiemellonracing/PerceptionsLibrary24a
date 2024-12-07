cmake_minimum_required (VERSION 3.10)
# Define the project
project(PerceptionsLibrary24a)
# GoogleTest requires at least C++11
set(CMAKE_CXX_STANDARD 11)
# Add definition for math library
add_library(math src/math.cc)
# Add definition for the PerceptionsLibrary24a binary
add_executable(PerceptionsLibrary24a src/main.cc)
target_link_libraries(PerceptionsLibrary24a math)
# Third-party library
include(ExternalProject)
ExternalProject_Add(googletest
    PREFIX “${CMAKE_BINARY_DIR}/lib”
    GIT_REPOSITORY “https://github.com/google/googletest.git”
    GIT_TAG “main”
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/lib/installed
)
# Prevent build on all targets build
set_target_properties(googletest PROPERTIES EXCLUDE_FROM_ALL TRUE)
# Define ${CMAKE_INSTALL_...} variables
include(GNUInstallDirs)
# Specify where third-party libraries are located
link_directories(${CMAKE_BINARY_DIR}/lib/installed/${CMAKE_INSTALL_LIBDIR})
include_directories(${CMAKE_BINARY_DIR}/lib/installed/${CMAKE_INSTALL_INCLUDEDIR})
# This is required for googletest
find_package(Threads REQUIRED)
find_package(PCL 1.14 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 3.4 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

# Test
add_executable(math_test test/math_test.cc)
target_link_libraries(math_test math gtest Threads::Threads)
# Make sure third-party is built before executable
add_dependencies(math_test googletest)
set_target_properties(math_test PROPERTIES EXCLUDE_FROM_ALL TRUE)

# PCL stuff
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(MY_GRAND_PROJECT)
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcd_write_test pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})
