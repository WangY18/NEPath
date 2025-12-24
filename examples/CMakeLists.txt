cmake_minimum_required(VERSION 3.15)
project(NEPath_Examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------------------------------
# 判断是否已安装 NEPath
# -------------------------------
# 用户可以通过 CMAKE_PREFIX_PATH 指定安装路径
find_package(NEPath QUIET)

if(NOT NEPath_FOUND)
    message(STATUS "NEPath not found via find_package, building from source...")
    # 假设 examples/CMakeLists.txt 在 examples/ 下，NEPath 源码在上一级
    add_subdirectory(.. ${CMAKE_CURRENT_BINARY_DIR}/NEPath_build)
else()
    message(STATUS "Found installed NEPath: ${NEPath_VERSION}")
endif()

# demo 列表
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
    demo_IQOP_Ipopt_CFS
    demo_IQOP_Ipopt_DFS
    demo_IQOP_gurobi
    demo_IQOP_gurobi_CFS
)

foreach(demo ${DEMO_LIST})
    add_executable(${demo} ${CMAKE_CURRENT_SOURCE_DIR}/${demo}.cpp)
        target_include_directories(${demo} PRIVATE
        ${CMAKE_SOURCE_DIR}/../include
    )
    target_link_libraries(${demo} PRIVATE NEPath)
endforeach()
