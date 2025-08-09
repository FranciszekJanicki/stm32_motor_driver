cmake_minimum_required(VERSION 4.0)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/build.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/target.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/submodules.cmake)

project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)

add_subdirectory(${MAIN_DIR})
add_subdirectories(${COMPONENTS_DIR})
add_subdirectories(${SUBMODULES_DIR})