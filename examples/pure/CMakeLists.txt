#examples / pure / CMakeLists.txt
cmake_minimum_required(VERSION 3.15)

# Project name for this directory (examples)
project(NEPath_Examples_Pure LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories for library headers
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

# Library source files
set(NEPATH_SOURCES
    ${CMAKE_SOURCE_DIR}/src/Basic.cpp
    ${CMAKE_SOURCE_DIR}/src/Connector.cpp
    ${CMAKE_SOURCE_DIR}/src/ContourParallel.cpp
    ${CMAKE_SOURCE_DIR}/src/Curve.cpp
    ${CMAKE_SOURCE_DIR}/src/DirectionParallel.cpp
    ${CMAKE_SOURCE_DIR}/src/FileAgent.cpp
    ${CMAKE_SOURCE_DIR}/src/NEPathPlanner.cpp
    ${CMAKE_SOURCE_DIR}/src/NonEquidistant.cpp
    ${CMAKE_SOURCE_DIR}/src/path.cpp
    ${CMAKE_SOURCE_DIR}/src/PlanningOptions.cpp
    ${CMAKE_SOURCE_DIR}/external/clipper/clipper.cpp
)

# Create executable for demo
add_executable(demo_Raster
    demo_Raster.cpp
    ${NEPATH_SOURCES}
)

# Optionally link system libraries if needed
# target_link_libraries(demo_Raster PRIVATE ...)

# Optional: print build configuration
message(STATUS "Building demo_Raster without IPOPT/Gurobi")
message(STATUS "Include directories: ${CMAKE_SOURCE_DIR}/include")
