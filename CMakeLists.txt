cmake_minimum_required(VERSION 3.15)
project(NEPath_Demos LANGUAGES CXX)

# Build configuration
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
message(STATUS "============= Build Configuration =============")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")

# Options to enable optional dependencies
option(IncludeIpopt "Enable IPOPT support" ON) # IPOPT: ON / OFF
option(IncludeGurobi "Enable Gurobi support" OFF) # Gurobi: ON / OFF

# Compiler definitions: IPOPT
if(IncludeIpopt)
    message(STATUS "Enable IPOPT support")
    add_compile_definitions(IncludeIpopt=1)

    find_package(PkgConfig) # Apply PkgConfig to find IPOPT
    if(PkgConfig_FOUND)
        pkg_check_modules(IPOPT ipopt)
    endif()

    if(DEFINED IPOPT_PKG_FOUND AND IPOPT_PKG_FOUND)
        message(STATUS "Found IPOPT via pkg-config")
    else()
        # Fallback to environment variable or user-provided path
        if(NOT DEFINED IPOPT_ROOT)
            if(DEFINED ENV{IPOPT_ROOT})
                message(STATUS "Using IPOPT_ROOT from environment: $ENV{IPOPT_ROOT}")
                set(IPOPT_ROOT "$ENV{IPOPT_ROOT}")
            else()
                message(FATAL_ERROR "IncludeIpopt=ON but IPOPT not found. Please set IPOPT_ROOT environment variable or pass -DIPOPT_ROOT=/path/to/ipopt")
            endif()
        endif()
        set(IPOPT_INCLUDE_DIRS "${IPOPT_ROOT}/include")
        set(IPOPT_LIBRARY_DIRS "${IPOPT_ROOT}/lib")
        set(IPOPT_LIBRARIES ipopt)
    endif()
    
    set(IPOPT_INCLUDE_DIRS ${IPOPT_INCLUDE_DIRS})
    set(IPOPT_LIBRARY_DIRS ${IPOPT_LIBRARY_DIRS})
    set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES})
    message(STATUS "IPOPT Include Dirs: ${IPOPT_INCLUDE_DIRS}")
    message(STATUS "IPOPT Library Dirs: ${IPOPT_LIBRARY_DIRS}")
    message(STATUS "IPOPT Libraries: ${IPOPT_LIBRARIES}")
    include_directories("${IPOPT_INCLUDE_DIRS}/coin")
    link_directories("${IPOPT_LIBRARY_DIRS}/coin")
else()
    message(STATUS "Disable IPOPT support")
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

set(DEMO_LIST
    demo_Raster
    demo_Zigzag
    demo_CP
    demo_CP_CFS
    demo_CP_DFS
    demo_tool_compensate
    demo_underfill
    demo_sharpcorner
    demo_IQOP_Ipopt
)

foreach(demo ${DEMO_LIST})
    message(STATUS "Building demo: ${demo}")
    add_executable(${demo}
        examples/${demo}.cpp
        ${NEPATH_SOURCES}
    ) # Create test_demos executable
    if(IncludeIpopt)
        target_link_libraries(${demo}
            ${IPOPT_LIBRARIES}
            ${CMAKE_DL_LIBS}
        ) # Link IPOPT libraries
        target_link_directories(${demo} PRIVATE
            ${IPOPT_LIBRARY_DIRS}
        ) # Add IPOPT link directories
    endif()
endforeach()

# if(IncludeIpopt)
    # target_link_libraries(test_demos
    #     ${IPOPT_LIBRARIES}
    #     ${CMAKE_DL_LIBS}
    # ) # Link IPOPT libraries
#     target_link_directories(test_demos PRIVATE
#         ${IPOPT_LIBRARY_DIRS}
#     ) # Add IPOPT link directories
# endif()

message(STATUS "=======================================")