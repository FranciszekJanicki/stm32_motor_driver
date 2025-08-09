add_library(stm32l4xx STATIC)

set(STM32L4XX_DRIVERS "${SUBMODULES_DIR}/STM32CubeL4/Drivers")
set(STM32L4XX_HAL_DRIVER "${STM32L4XX_DRIVERS}/STM32L4xx_HAL_Driver")
set(STM32L4XX_CMSIS_DRIVER "${STM32L4XX_DRIVERS}/CMSIS")

target_sources(stm32l4xx PRIVATE 
    stm32l4xx_it.c
    stm32l4xx_hal_msp.c
    stm32l4xx_system.c
    stm32l4xx_startup.s
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_i2c.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_i2c_ex.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_rcc.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_rcc_ex.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_flash.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_flash_ex.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_flash_ramfunc.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_gpio.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_dma.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_dma_ex.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_pwr.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_pwr_ex.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_cortex.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_exti.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_tim.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_tim_ex.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_uart.c
    ${STM32L4XX_HAL_DRIVER}/Src/stm32l4xx_hal_uart_ex.c
)

target_include_directories(stm32l4xx PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${STM32L4XX_HAL_DRIVER}/Inc
    ${STM32L4XX_HAL_DRIVER}/Inc/Legacy
    ${STM32L4XX_CMSIS_DRIVER}/Device/ST/STM32L4xx/Include
    ${STM32L4XX_CMSIS_DRIVER}/Include
)

target_compile_options(stm32l4xx PUBLIC
    -std=c23
    -Wall
    -Wextra
    -Wconversion
    -Wshadow
    -Wpedantic
    -Wnarrowing
    -Waddress
    -pedantic
    -Wdeprecated
    -Wsign-conversion
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wcast-align
    -Wformat=2
    -Wformat-security
    -Wmissing-prototypes
    -Wmissing-declarations
    -Wstrict-prototypes
    -Wold-style-definition
    -Wundef
    -Wvla
    -Wpointer-arith
    -Wstrict-aliasing=2
)

target_compile_definitions(stm32l4xx PUBLIC
	USE_HAL_DRIVER 
	STM32L476xx
    $<$<CONFIG:Debug>:DEBUG>
)