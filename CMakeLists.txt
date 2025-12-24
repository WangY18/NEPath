cmake_minimum_required(VERSION 3.15)

project(
    NEPath
    VERSION 1.0.0
    LANGUAGES CXX
)

# ============================================================
# Build type (default: Release, do not override user choice)
# ============================================================
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Build type (default: Release)" FORCE)
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "RelWithDebInfo" "MinSizeRel")

# ============================================================
# Options
# ============================================================
option(NEPATH_ENABLE_IPOPT  "Enable IPOPT support"  ON)
option(NEPATH_ENABLE_GUROBI "Enable Gurobi support" ON)

message(STATUS "========== NEPath Configuration ==========")
message(STATUS "Build type        : ${CMAKE_BUILD_TYPE}")
message(STATUS "IPOPT support     : ${NEPATH_ENABLE_IPOPT}")
message(STATUS "Gurobi support    : ${NEPATH_ENABLE_GUROBI}")

# ============================================================
# Compiler settings
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wno-deprecated-declarations
        -Wno-sign-compare
    )
elseif(MSVC)
    add_compile_options(/W4 /wd4996)
endif()

# ============================================================
# Dependencies (library-level)
# ============================================================

# ---------- IPOPT ----------
if(NEPATH_ENABLE_IPOPT)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(IPOPT QUIET ipopt)
    endif()

    if(IPOPT_FOUND)
        add_library(IPOPT INTERFACE)
        target_include_directories(IPOPT INTERFACE
            ${IPOPT_INCLUDE_DIRS}
        )
        target_link_libraries(IPOPT INTERFACE
            ${IPOPT_LIBRARIES}
            ${CMAKE_DL_LIBS}
        )
        message(STATUS "Found IPOPT via pkg-config")
    else()
        if(NOT DEFINED IPOPT_ROOT AND DEFINED ENV{IPOPT_ROOT})
            set(IPOPT_ROOT "$ENV{IPOPT_ROOT}")
        endif()

        if(NOT IPOPT_ROOT)
            message(FATAL_ERROR
                "NEPATH_ENABLE_IPOPT=ON but IPOPT not found. "
                "Set IPOPT_ROOT or disable IPOPT.")
        endif()

        add_library(IPOPT INTERFACE)
        target_include_directories(IPOPT INTERFACE
            ${IPOPT_ROOT}/include
            ${IPOPT_ROOT}/include/coin
        )
        target_link_directories(IPOPT INTERFACE
            ${IPOPT_ROOT}/lib
        )
        target_link_libraries(IPOPT INTERFACE
            ipopt
            ${CMAKE_DL_LIBS}
        )

        message(STATUS "Using IPOPT from ${IPOPT_ROOT}")
    endif()
endif()

# ---------- Gurobi ----------
if(NEPATH_ENABLE_GUROBI)
    if(NOT DEFINED GUROBI_HOME AND DEFINED ENV{GUROBI_HOME})
        set(GUROBI_HOME "$ENV{GUROBI_HOME}")
    endif()

    if(NOT DEFINED GUROBI_VERSION AND DEFINED ENV{GUROBI_VERSION})
        set(GUROBI_VERSION "$ENV{GUROBI_VERSION}")
    endif()

    if(NOT GUROBI_HOME OR NOT GUROBI_VERSION)
        message(FATAL_ERROR
            "NEPATH_ENABLE_GUROBI=ON but GUROBI_HOME or GUROBI_VERSION not set.")
    endif()

    add_library(GUROBI INTERFACE)
    target_include_directories(GUROBI INTERFACE
        ${GUROBI_HOME}/include
    )
    target_link_directories(GUROBI INTERFACE
        ${GUROBI_HOME}/lib
    )
    target_link_libraries(GUROBI INTERFACE
        gurobi_c++
        gurobi${GUROBI_VERSION}
    )

    message(STATUS "Using Gurobi ${GUROBI_VERSION} from ${GUROBI_HOME}")
endif()

# ============================================================
# NEPath library
# ============================================================

add_library(NEPath
    src/NEPath/Basic.cpp
    src/NEPath/Connector.cpp
    src/NEPath/ContourParallel.cpp
    src/NEPath/Curve.cpp
    src/NEPath/DirectionParallel.cpp
    src/NEPath/FileAgent.cpp
    src/NEPath/NEPathPlanner.cpp
    src/NEPath/path.cpp
    src/NEPath/NonEquidistant.cpp
    external/clipper/clipper.cpp
)

add_library(NEPath::NEPath ALIAS NEPath)

# Public headers
target_include_directories(NEPath
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/external/clipper
)

# Suppress warnings from Clipper
set_source_files_properties(
    external/clipper/clipper.cpp
    PROPERTIES COMPILE_FLAGS "-w"
)

# Optional dependencies
if(NEPATH_ENABLE_IPOPT)
    target_compile_definitions(NEPath PUBLIC IncludeIpopt=1)
    target_link_libraries(NEPath PUBLIC IPOPT)
else()
    target_compile_definitions(NEPath PUBLIC IncludeIpopt=0)
endif()

if(NEPATH_ENABLE_GUROBI)
    target_compile_definitions(NEPath PUBLIC IncludeGurobi=1)
    target_link_libraries(NEPath PUBLIC GUROBI)
else()
    target_compile_definitions(NEPath PUBLIC IncludeGurobi=0)
endif()

message(STATUS "=======================================")
