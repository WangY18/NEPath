cmake_minimum_required(VERSION 3.15)
project(NEPath_Python LANGUAGES CXX)

# ============================================================
# C++ standard
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# Options
# ============================================================
option(NEPATH_ENABLE_IPOPT  "Enable IPOPT support"  ON)
option(NEPATH_ENABLE_GUROBI "Enable Gurobi support" ON)

if(DEFINED ENV{NEPATH_ENABLE_IPOPT})
    set(NEPATH_ENABLE_IPOPT $ENV{NEPATH_ENABLE_IPOPT} CACHE BOOL "Enable IPOPT support from environment variable" FORCE)
endif()

if(DEFINED ENV{NEPATH_ENABLE_GUROBI})
    set(NEPATH_ENABLE_GUROBI $ENV{NEPATH_ENABLE_GUROBI} CACHE BOOL "Enable Gurobi support from environment variable" FORCE)
endif()

message(STATUS "========== NEPath Configuration ==========")
message(STATUS "Build type        : ${CMAKE_BUILD_TYPE}")
message(STATUS "IPOPT support     : ${NEPATH_ENABLE_IPOPT}")
message(STATUS "Gurobi support    : ${NEPATH_ENABLE_GUROBI}")

# ============================================================
# Find Python and nanobind
# ============================================================
find_package(Python COMPONENTS Interpreter Development REQUIRED)
message(STATUS "Found Python ${Python_VERSION_STRING} at ${Python_INCLUDE_DIRS}")
find_package(nanobind CONFIG REQUIRED)
if(NOT nanobind_FOUND)
    message(FATAL_ERROR "Nanobind not found via CMake, please set nanobind_DIR or use system install.")
else()
    message(STATUS "nanobind_FOUND: ${nanobind_FOUND}")
    message(STATUS "nanobind_INCLUDE_DIRS: ${nanobind_INCLUDE_DIRS}")
    message(STATUS "nanobind_LIBRARIES: ${nanobind_LIBRARIES}")
    message(STATUS "nanobind_DIR: ${nanobind_DIR}")
endif()

# ============================================================
# Include NEPath headers
# ============================================================
include_directories(
    ${CMAKE_SOURCE_DIR}/../../include  # Path to NEPath public headers
)


# ============================================================
# Build Python extension module
# ============================================================
nanobind_add_module(nepath_py
    STABLE_ABI 
    NB_STATIC
    NEPath/nepath_py.cpp
    ../../src/NEPath/Basic.cpp
    ../../src/NEPath/Connector.cpp
    ../../src/NEPath/ContourParallel.cpp
    ../../src/NEPath/Curve.cpp
    ../../src/NEPath/DirectionParallel.cpp
    ../../src/NEPath/FileAgent.cpp
    ../../src/NEPath/NEPathPlanner.cpp
    ../../src/NEPath/path.cpp
    ../../src/NEPath/NonEquidistant.cpp
    ../../external/clipper/clipper.cpp
)


target_include_directories(nepath_py PRIVATE
    ../../src
    ../../external
    ${CMAKE_SOURCE_DIR}/../../include
    ${nanobind_DIR}/../include
    ${Python_INCLUDE_DIRS}
)

# target_link_libraries(nepath_py PRIVATE nanobind)


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

# Optional dependencies
if(NEPATH_ENABLE_IPOPT)
    target_compile_definitions(nepath_py PUBLIC IncludeIpopt=1)
    # target_link_libraries(nepath_py PUBLIC IPOPT)
    target_link_libraries(nepath_py PRIVATE $<$<BOOL:${NEPATH_ENABLE_IPOPT}>:IPOPT>)
    set(NEPATH_ENABLE_IPOPT_BOOL True)

else()
    target_compile_definitions(nepath_py PUBLIC IncludeIpopt=0)
    set(NEPATH_ENABLE_IPOPT_BOOL False)
endif()

if(NEPATH_ENABLE_GUROBI)
    target_compile_definitions(nepath_py PUBLIC IncludeGurobi=1)
    # target_link_libraries(nepath_py PUBLIC GUROBI)
    target_link_libraries(nepath_py PRIVATE  $<$<BOOL:${NEPATH_ENABLE_GUROBI}>:GUROBI>)
    set(NEPATH_ENABLE_GUROBI_BOOL True)
else()
    target_compile_definitions(nepath_py PUBLIC IncludeGurobi=0)
    set(NEPATH_ENABLE_GUROBI_BOOL False)
endif()

# Remove default 'lib' prefix (so Python can import the module)
set_target_properties(nepath_py PROPERTIES
    PREFIX ""
    OUTPUT_NAME "_nepath"  # Module name must match NB_MODULE(_nepath, m)
)

# ============================================================
# Install Python module
# ============================================================
# Install the compiled module into Python site-packages
install(TARGETS nepath_py
    LIBRARY DESTINATION ${Python_SITEARCH}
)

# Install __init__.py to allow 'import NEPath'
install(TARGETS nepath_py LIBRARY DESTINATION ${Python_SITEARCH}/NEPath)
install(FILES 
    ${CMAKE_CURRENT_SOURCE_DIR}/NEPath/__init__.py
    ${CMAKE_CURRENT_SOURCE_DIR}/NEPath/_nepath.pyi
    DESTINATION
    ${Python_SITEARCH}/NEPath)

# Add config file for IncludeIpopt and IncludeGurobi
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/NEPathConfig.py.in"
    "${CMAKE_CURRENT_BINARY_DIR}/NEPathConfig.py"
    @ONLY
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/NEPathConfig.py"
        DESTINATION 
        ${Python_SITEARCH}/NEPath)