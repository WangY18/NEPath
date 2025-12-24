cmake_minimum_required(VERSION 3.15)
project(NEPath_Examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 假设用户在 examples/build 里执行 cmake ..
# add_subdirectory(..) 会先构建顶层 NEPath 库
add_subdirectory(.. ${CMAKE_CURRENT_BINARY_DIR}/NEPath_build)

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
    target_link_libraries(${demo} PRIVATE NEPath)
endforeach()
