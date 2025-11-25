cmake_minimum_required(VERSION 3.15)
project(NEPath_Demos LANGUAGES CXX)

# Build configuration
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find IPOPT
find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

# Compiler definitions
add_compile_definitions(
    IncludeIpopt=1          # Enable IPOPT support
    IncludeGurobi=0         # Disable Gurobi
    ENABLE_DIRECTION_PARALLEL  # Enable Raster/Zigzag support
)

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(
        -Wall
        -Wno-register           # Allow deprecated 'register' keyword (if any remain)
        -Wno-deprecated-declarations
        -Wno-infinite-recursion # Suppress FileAgent::mkdir warning
        -Wno-null-conversion    # Suppress path.cpp NULL conversion warning
        -Wno-pragma-once-outside-header  # Suppress path.cpp pragma once warning
    )
elseif(MSVC)
    add_compile_options(
        /W4
        /wd4996  # Disable deprecation warnings
    )
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/NEPath-master
    ${IPOPT_INCLUDE_DIRS}
)

# Collect NEPath source files
set(NEPATH_SOURCES
    NEPath-master/Basic.cpp
    NEPath-master/Connector.cpp
    NEPath-master/ContourParallel.cpp
    NEPath-master/Curve.cpp
    NEPath-master/DirectionParallel.cpp
    NEPath-master/FileAgent.cpp
    NEPath-master/NEPathPlanner.cpp
    NEPath-master/PlanningOptions.cpp
    NEPath-master/clipper.cpp
    NEPath-master/path.cpp
    NEPath-master/NonEquidistant.cpp  # Included with IPOPT support
)

# Demo sources
set(DEMO_SOURCES
    demos.cpp
)

# Create test_demos executable
add_executable(test_demos
    main.cpp
    ${NEPATH_SOURCES}
    ${DEMO_SOURCES}
)

# Link IPOPT libraries
target_link_libraries(test_demos
    ${IPOPT_LIBRARIES}
    ${CMAKE_DL_LIBS}
)

# Add IPOPT link directories
target_link_directories(test_demos PRIVATE
    ${IPOPT_LIBRARY_DIRS}
)

message(STATUS "============= Build Configuration =============")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "IPOPT Include Dirs: ${IPOPT_INCLUDE_DIRS}")
message(STATUS "IPOPT Library Dirs: ${IPOPT_LIBRARY_DIRS}")
message(STATUS "IPOPT Libraries: ${IPOPT_LIBRARIES}")
message(STATUS "=======================================")
