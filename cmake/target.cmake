include_guard(GLOBAL)

include(${CMAKE_CURRENT_LIST_DIR}/common.cmake)

if(NOT DEFINED STM32_FAMILY)
    message(FATAL_ERROR "Please set STM32_FAMILY (e.g. -DSTM32_FAMILY=stm32l4xx)")
else()
    set(STM32_CMAKE_FILE "${COMPONENTS_DIR}/${STM32_FAMILY}/${STM32_FAMILY}.cmake")
endif()

if(NOT EXISTS ${STM32_CMAKE_FILE})
    message(WARNING "Unsupported STM32 family: ${STM32_FAMILY}")
else()
    include(${STM32_CMAKE_FILE})
endif()