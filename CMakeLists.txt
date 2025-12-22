cmake_minimum_required(VERSION 3.15)
project(NEPath_Demos LANGUAGES CXX)

# Build configuration
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options to enable optional dependencies
option(IncludeIpopt "Enable IPOPT support" OFF)
option(IncludeGurobi "Enable Gurobi support" OFF)

# Compiler definitions: IPOPT
if(IncludeIpopt)
    add_compile_definitions(IncludeIpopt=1)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(IPOPT REQUIRED ipopt)
else()
    add_compile_definitions(IncludeIpopt=0)
endif()

# Compiler definitions: Gurobi
if(IncludeGurobi)
    add_compile_definitions(IncludeGurobi=1)
else()
    add_compile_definitions(IncludeGurobi=0)
endif()

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(
        -Wall
        -Wno-register           # Allow deprecated 'register' keyword (if any remain)
        -Wno-deprecated-declarations
        -Wno-infinite-recursion # Suppress FileAgent::mkdir warning
        -Wno-null-conversion    # Suppress path.cpp NULL conversion warning
        -Wno-pragma-once-outside-header  # Suppress path.cpp pragma once warning
        -Wno-sign-compare      # Suppress sign-compare warnings in NEPath sources
    )
elseif(MSVC)
    add_compile_options(
        /W4
        /wd4996  # Disable deprecation warnings
    )
endif()

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/external/clipper
    # ${IPOPT_INCLUDE_DIRS}
)

# Collect NEPath source files
set(NEPATH_SOURCES
    src/Basic.cpp
    src/Connector.cpp
    src/ContourParallel.cpp
    src/Curve.cpp
    src/DirectionParallel.cpp
    src/FileAgent.cpp
    src/NEPathPlanner.cpp
    src/PlanningOptions.cpp
    src/path.cpp
    external/clipper/clipper.cpp
    src/NonEquidistant.cpp
)

# Eliminate warnings from Clipper library
set_source_files_properties(external/clipper/clipper.cpp PROPERTIES
    COMPILE_FLAGS "-w"
)

# Create test_demos executable
add_executable(demo_Raster
    examples/demo_Raster.cpp
    ${NEPATH_SOURCES}
)
add_executable(demo_Zigzag
    examples/demo_Zigzag.cpp
    ${NEPATH_SOURCES}
)
add_executable(demo_CP
    examples/demo_CP.cpp
    ${NEPATH_SOURCES}
)
add_executable(demo_CP_CFS
    examples/demo_CP_CFS.cpp
    ${NEPATH_SOURCES}
)
add_executable(demo_CP_DFS
    examples/demo_CP_DFS.cpp
    ${NEPATH_SOURCES}
)
add_executable(demo_tool_compensate
    examples/demo_tool_compensate.cpp
    ${NEPATH_SOURCES}
)
add_executable(demo_underfill
    examples/demo_underfill.cpp
    ${NEPATH_SOURCES}
)
add_executable(demo_sharpcorner
    examples/demo_sharpcorner.cpp
    ${NEPATH_SOURCES}
)



if(IncludeIpopt)
    target_link_libraries(test_demos
        ${IPOPT_LIBRARIES}
        ${CMAKE_DL_LIBS}
    ) # Link IPOPT libraries
    target_link_directories(test_demos PRIVATE
        ${IPOPT_LIBRARY_DIRS}
    ) # Add IPOPT link directories
endif()

message(STATUS "============= Build Configuration =============")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "=======================================")
if(IncludeIpopt)
    message(STATUS "IPOPT Include Dirs: ${IPOPT_INCLUDE_DIRS}")
    message(STATUS "IPOPT Library Dirs: ${IPOPT_LIBRARY_DIRS}")
    message(STATUS "IPOPT Libraries: ${IPOPT_LIBRARIES}")
endif()