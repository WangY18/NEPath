cmake_minimum_required(VERSION 3.15)

project(
    NEPath
    VERSION 1.0.0
    LANGUAGES CXX
)

# ============================================================
# Build type (default: Release, do not override user choice)
# ============================================================
if(NOT DEFINED CMAKE_CONFIGURATION_TYPES)
    if(NOT DEFINED CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (default: Release)" FORCE)
    endif()

    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# ============================================================
# Options
# ============================================================
option(NEPATH_ENABLE_IPOPT  "Enable IPOPT support"  ON)
option(NEPATH_ENABLE_GUROBI "Enable Gurobi support" ON)

if(DEFINED ENV{NEPATH_ENABLE_IPOPT})
    set(NEPATH_ENABLE_IPOPT ENV{NEPATH_ENABLE_IPOPT} CACHE BOOL "Enable IPOPT support from environment variable")
endif()

if(DEFINED ENV{NEPATH_ENABLE_GUROBI})
    set(NEPATH_ENABLE_GUROBI ENV{NEPATH_ENABLE_GUROBI} CACHE BOOL "Enable Gurobi support from environment variable")
endif()

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
    add_compile_options(
        /W4
        /wd4996
        /wd4251
    )
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
    if(MSVC)
        # Windows MSVC
        # Find all gurobi_c++ libs
        file(GLOB GUROBI_CPP_LIBS "${GUROBI_HOME}/lib/gurobi_c++*.lib")
        set(GUROBI_CPP_LIB "")
        # Determine the unique library based on the current configuration
        foreach(lib ${GUROBI_CPP_LIBS})
            get_filename_component(libname ${lib} NAME)
            if("$<CONFIG:Debug>" STREQUAL "Debug")
                if(libname MATCHES "mdd[0-9]+\\.lib$")
                    set(GUROBI_CPP_LIB ${lib})
                    break()
                endif()
            else() # Release
                if(libname MATCHES "md[0-9]+\\.lib$")
                    set(GUROBI_CPP_LIB ${lib})
                    break()
                endif()
            endif()
        endforeach()
        if(NOT GUROBI_CPP_LIB)
            message(FATAL_ERROR "Cannot find matching Gurobi C++ library for ${CMAKE_BUILD_TYPE}")
        else()
            message(STATUS "Using Gurobi C++ library: ${GUROBI_CPP_LIB}")
        endif()
        target_link_libraries(GUROBI INTERFACE
            ${GUROBI_CPP_LIB}
            "${GUROBI_HOME}/lib/gurobi${GUROBI_VERSION}.lib"
        )
    else()
        # Linux / macOS
        target_link_libraries(GUROBI INTERFACE
            gurobi_c++
            gurobi${GUROBI_VERSION}
        )
    endif()

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
        ${CMAKE_SOURCE_DIR}/external
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

# ============================================================
# Install NEPath library and headers
# ============================================================

# 1. Headers
install(DIRECTORY include/ 
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY external/clipper/
    DESTINATION include/clipper
    FILES_MATCHING PATTERN "*.hpp"
) # Install Clipper headers

# 2. Library
set_target_properties(NEPath PROPERTIES
    EXPORT_NAME NEPath
    PUBLIC_HEADER "${CMAKE_SOURCE_DIR}/include/NEPath/NEPath.h"
)

install(TARGETS NEPath
    EXPORT NEPathTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# 3. Export CMake package config
install(EXPORT NEPathTargets
    FILE NEPathTargets.cmake
    NAMESPACE NEPath::
    DESTINATION lib/cmake/NEPath
)

if(NEPATH_ENABLE_IPOPT)
    install(TARGETS IPOPT
        EXPORT NEPathTargets
    )
endif()

if(NEPATH_ENABLE_GUROBI)
    install(TARGETS GUROBI
        EXPORT NEPathTargets
    )
endif()

# 4. Package version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/NEPathConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/NEPathConfigVersion.cmake"
    DESTINATION lib/cmake/NEPath
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/NEPathConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/NEPathConfig.cmake"
    @ONLY
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/NEPathConfig.cmake"
    DESTINATION lib/cmake/NEPath
)