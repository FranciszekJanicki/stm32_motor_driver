include_guard(GLOBAL)

include(${CMAKE_CURRENT_LIST_DIR}/common.cmake)

set(CUBEMX_CMAKE_FILE ${CUBEMX_DIR}/../gcc-arm-none-eabi.cmake)

if(EXISTS ${CUBEMX_CMAKE_FILE})
    include(${CUBEMX_CMAKE_FILE})
else()
    message(WARNING "CubeMX CMake file not found: ${CUBEMX_CMAKE_FILE}")
endif()
