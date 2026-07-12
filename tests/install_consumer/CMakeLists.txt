cmake_minimum_required(VERSION 3.15)
project(NEPathInstallConsumer LANGUAGES CXX)

find_package(NEPath CONFIG REQUIRED)

add_executable(nepath_install_consumer main.cpp)
target_link_libraries(nepath_install_consumer PRIVATE NEPath::NEPath)
