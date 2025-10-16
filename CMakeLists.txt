cmake_minimum_required(VERSION 3.10)
project(BasicRegionMerging CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O3")

# Add debug symbols in Debug mode
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# Find PCL (Point Cloud Library)
find_package(PCL 1.8 REQUIRED COMPONENTS common io kdtree features)

# Include directories
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# Add PCL definitions
add_definitions(${PCL_DEFINITIONS})

# Source files
set(SOURCES
    main.cpp
    graph/region_adjacency_graph.cpp
)

# Create executable
add_executable(region_merging ${SOURCES})

# Link PCL libraries
target_link_libraries(region_merging ${PCL_LIBRARIES})

# Print build information
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "C++ flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "PCL version: ${PCL_VERSION}")
message(STATUS "PCL include dirs: ${PCL_INCLUDE_DIRS}")
