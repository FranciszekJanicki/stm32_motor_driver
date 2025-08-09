add_executable(main)

target_sources(main PRIVATE 
    main.c
    syscalls.c
    sysmem.c
    i2c.c
    tim.c
    usart.c
    gpio.c
)

target_link_libraries(main PRIVATE
    a4988
    step_motor
    as5600
    motor_driver
    stm32l4xx
)

target_compile_options(main PUBLIC
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
