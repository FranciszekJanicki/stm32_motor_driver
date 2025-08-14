cmake_minimum_required(VERSION 4.0)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/cubemx.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/build.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/common.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/submodules.cmake)

project(${CMAKE_PROJECT_NAME} LANGUAGES C CXX ASM)
message(STATUS "Project name: ${CMAKE_PROJECT_NAME}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

add_subdirectory(${MAIN_DIR})
add_subdirectory(${CUBEMX_DIR})
add_subdirectories(${COMPONENTS_DIR})
add_subdirectories(${SUBMODULES_DIR})

target_compile_options(stm32cubemx INTERFACE -w)
