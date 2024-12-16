cmake_minimum_required(VERSION 3.18)
project(GraceAndConrad LANGUAGES CXX CUDA)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find CUDA
find_package(CUDA REQUIRED)

# Add the CSV parser library
set(CSV_PARSER_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/external/csv-parser/include")
include_directories(${CSV_PARSER_INCLUDE_DIR})

# Add CUDA files
enable_language(CUDA)

# Set the sources to pull from
set(SOURCES
    main.cpp
    GraceAndConrad.cu
    ColorCones.cu
    DBscan.cu

)

# Add executable and link CUDA
add_executable(perceptions ${SOURCES})

# Link CUDA
target_link_libraries(perceptions ${CUDA_LIBRARIES})
target_include_directories(perceptions PRIVATE ${CUDA_INCLUDE_DIRS})

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --extended-lambda")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr")