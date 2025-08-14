#include "usart.h"
#include <stdio.h>

// #include "a4988.h"
// #include "as5600.h"
// #include "i2c.h"
// #include "step_motor.h"
// #include "stm32l476xx.h"
// #include "stm32l4xx_hal.h"
// #include "tim.h"
// #include <stdbool.h>
// #include <stdint.h>
// #include <string.h>

// static bool frequency_to_prescaler_and_period(uint32_t frequency_hz,
//                                               uint32_t clock_hz,
//                                               uint32_t max_prescaler,
//                                               uint32_t max_period,
//                                               uint32_t* prescaler,
//                                               uint32_t* period)
// {
//     if (frequency_hz == 0U || !prescaler || !period) {
//         return false;
//     }

//     uint32_t temp_prescaler = 0U;
//     uint32_t temp_period = clock_hz / frequency_hz;

//     while (temp_period > max_period && temp_prescaler < max_prescaler) {
//         temp_prescaler++;
//         temp_period = clock_hz / ((temp_prescaler + 1U) * frequency_hz);
//     }
//     if (temp_period > max_period) {
//         temp_period = max_period;
//         temp_prescaler = (clock_hz / (temp_period * frequency_hz)) - 1U;
//     }
//     if (temp_prescaler > max_prescaler) {
//         temp_prescaler = max_prescaler;
//     }

//     *prescaler = temp_prescaler;
//     *period = temp_period;

//     return true;
// }

// static a4988_err_t a4988_pwm_start(void* user)
// {
//     HAL_TIM_PWM_Start(&TIM2_Handle, TIM_CHANNEL_1);

//     return A4988_ERR_OK;
// }

// static a4988_err_t a4988_pwm_stop(void* user)
// {
//     HAL_TIM_PWM_Stop(&TIM2_Handle, TIM_CHANNEL_1);

//     return A4988_ERR_OK;
// }

// static a4988_err_t a4988_pwm_set_frequency(void* user, uint32_t frequency)
// {
//     uint32_t clock_hz = HAL_RCC_GetPCLK1Freq();
//     if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
//         clock_hz *= 2;
//     }

//     uint32_t period;
//     uint32_t prescaler;
//     bool result = frequency_to_prescaler_and_period(frequency,
//                                                     clock_hz,
//                                                     0xFFFFU,
//                                                     0xFFFFU,
//                                                     &prescaler,
//                                                     &period);

//     if (result && period < 0xFFFFU && prescaler < 0xFFFFU) {
//         uint32_t tick_hz = clock_hz / (prescaler + 1);
//         uint32_t compare = (tick_hz / 1000000) * 5; // 5 us pulse
//         if (compare == 0) {
//             compare = 1;
//         }
//         if (compare > period) {
//             compare = period;
//         }

//         __HAL_TIM_DISABLE(&TIM2_Handle);
//         __HAL_TIM_SET_COUNTER(&TIM2_Handle, 0U);
//         __HAL_TIM_SET_PRESCALER(&TIM2_Handle, prescaler);
//         __HAL_TIM_SET_AUTORELOAD(&TIM2_Handle, period);
//         __HAL_TIM_SET_COMPARE(&TIM2_Handle, TIM_CHANNEL_1, compare);
//         __HAL_TIM_ENABLE(&TIM2_Handle);

//         printf("FREQ: %lu, PSC: %lu, CP: %lu, CMP: %lu\n\r",
//                frequency,
//                prescaler,
//                period,
//                compare);
//     }

//     return A4988_ERR_OK;
// }

// static a4988_err_t a4988_gpio_write_pin(void* user, uint32_t pin, bool state)
// {
//     HAL_GPIO_WritePin(GPIOA, (uint16_t)pin, (GPIO_PinState)state);

//     return A4988_ERR_OK;
// }

// static step_motor_err_t step_motor_device_set_frequency(void* user,
//                                                         uint32_t frequency)
// {
//     a4988_t* a4988 = (a4988_t*)user;

//     a4988_set_frequency(a4988, frequency);

//     return STEP_MOTOR_ERR_OK;
// }

// static step_motor_err_t step_motor_device_set_direction(
//     void* user,
//     step_motor_direction_t direction)
// {
//     a4988_t* a4988 = (a4988_t*)user;

//     a4988_set_direction(a4988, (a4988_direction_t)direction);

//     return STEP_MOTOR_ERR_OK;
// }

// static as5600_err_t as5600_bus_write_data(void* user,
//                                           uint8_t address,
//                                           uint8_t const* data,
//                                           size_t data_size)
// {
//     HAL_I2C_Mem_Write(&I2C1_Handle,
//                       AS5600_SLAVE_ADDRESS << 1U,
//                       address,
//                       1U,
//                       (uint8_t*)data,
//                       data_size,
//                       100);

//     return AS5600_ERR_OK;
// }

// static as5600_err_t as5600_bus_read_data(void* user,
//                                          uint8_t address,
//                                          uint8_t* data,
//                                          size_t data_size)
// {
//     HAL_I2C_Mem_Read(&I2C1_Handle,
//                      AS5600_SLAVE_ADDRESS << 1U,
//                      address,
//                      1U,
//                      data,
//                      data_size,
//                      100);

//     return AS5600_ERR_OK;
// }

// static as5600_err_t as5600_gpio_write_pin(void* user, uint32_t pin, bool
// state)
// {
//     HAL_GPIO_WritePin(user, (uint16_t)pin, (GPIO_PinState)state);

//     return AS5600_ERR_OK;
// }

// as5600_err_t as5600_init_chip(as5600_t* as5600,
//                               float32_t min_angle,
//                               float32_t max_angle)
// {
//     as5600_status_reg_t status;
//     as5600_err_t err = as5600_get_status_reg(as5600, &status);
//     if (err != AS5600_ERR_OK)
//         return err;

//     float32_t angle_range = (max_angle - min_angle);

//     uint16_t min_raw = (uint16_t)(min_angle / angle_range * 4095.0F);
//     uint16_t max_raw = (uint16_t)(max_angle / angle_range * 4095.0F);

//     as5600_zpos_reg_t zpos = {.zpos = min_raw & 0x0FFF};
//     err = as5600_set_zpos_reg(as5600, &zpos);
//     if (err != AS5600_ERR_OK)
//         return err;

//     as5600_mpos_reg_t mpos = {.mpos = max_raw & 0x0FFF};
//     err = as5600_set_mpos_reg(as5600, &mpos);
//     if (err != AS5600_ERR_OK)
//         return err;

//     as5600_conf_reg_t conf = {.wd = AS5600_WATCHDOG_OFF,
//                               .fth = AS5600_SLOW_FILTER_X16,
//                               .sf = AS5600_SLOW_FILTER_X16,
//                               .pwmf = AS5600_PWM_FREQUENCY_115HZ,
//                               .outs = AS5600_FAST_FILTER_THRESH_SLOW,
//                               .hyst = AS5600_HYSTERESIS_OFF,
//                               .pm = AS5600_POWER_MODE_NOM};
//     err = as5600_set_conf_reg(as5600, &conf);
//     if (err != AS5600_ERR_OK)
//         return err;

//     as5600_zmco_reg_t zmco;
//     err = as5600_get_zmco_reg(as5600, &zmco);
//     if (err != AS5600_ERR_OK)
//         return err;

//     return AS5600_ERR_OK;
// }

// void Error_Handler(void)
// {
//     __disable_irq();
//     while (1) {
//     }
// }

// void SystemClock_Config(void)
// {
//     RCC_OscInitTypeDef RCC_OscInit = {0};
//     RCC_ClkInitTypeDef RCC_ClkInit = {0};

//     if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) !=
//         HAL_OK) {
//         Error_Handler();
//     }

//     RCC_OscInit.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//     RCC_OscInit.HSIState = RCC_HSI_ON;
//     RCC_OscInit.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//     RCC_OscInit.PLL.PLLState = RCC_PLL_ON;
//     RCC_OscInit.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//     RCC_OscInit.PLL.PLLM = 1;
//     RCC_OscInit.PLL.PLLN = 10;
//     RCC_OscInit.PLL.PLLP = RCC_PLLP_DIV7;
//     RCC_OscInit.PLL.PLLQ = RCC_PLLQ_DIV2;
//     RCC_OscInit.PLL.PLLR = RCC_PLLR_DIV2;
//     if (HAL_RCC_OscConfig(&RCC_OscInit) != HAL_OK) {
//         Error_Handler();
//     }

//     RCC_ClkInit.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
//                             RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
//     RCC_ClkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//     RCC_ClkInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
//     RCC_ClkInit.APB1CLKDivider = RCC_HCLK_DIV1;
//     RCC_ClkInit.APB2CLKDivider = RCC_HCLK_DIV1;

//     if (HAL_RCC_ClockConfig(&RCC_ClkInit, FLASH_LATENCY_4) != HAL_OK) {
//         Error_Handler();
//     }

//     HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
//     HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
// }

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    GPIO_Init();
    UART2_Init();

    while (1) {
        puts("DUPA\n\r");
    }

    // MX_TIM2_Init();
    // MX_I2C1_Init();

    // a4988_t a4988;
    // a4988_initialize(
    //     &a4988,
    //     &(a4988_config_t){.pin_dir = 1 << 9},
    //     &(a4988_interface_t){.gpio_write_pin = a4988_gpio_write_pin,
    //                          .pwm_start = a4988_pwm_start,
    //                          .pwm_stop = a4988_pwm_stop,
    //                          .pwm_set_frequency =
    //                          a4988_pwm_set_frequency});

    // // as5600_t as5600;
    // // as5600_initialize(
    // //     &as5600,
    // //     &(as5600_config_t){.dir_pin = GPIO_PIN_2,
    // //                        .min_angle = 0.0F,
    // //                        .max_angle = 180.0F},
    // //     &(as5600_interface_t){.gpio_user = GPIOC,
    // //                           .gpio_write_pin = as5600_gpio_write_pin,
    // //                           .bus_write_data = as5600_bus_write_data,
    // //                           .bus_read_data = as5600_bus_read_data});

    // // as5600_init_chip(&as5600, 0.0F, 180.0F);

    // step_motor_t motor;
    // step_motor_initialize(
    //     &motor,
    //     &(step_motor_config_t){.min_speed = 10.0F,
    //                            .max_speed = 3000.0F,
    //                            .step_change = 1.8F},
    //     &(step_motor_interface_t){
    //         .device_user = &a4988,
    //         .device_set_frequency = step_motor_device_set_frequency,
    //         .device_set_direction = step_motor_device_set_direction},
    //     0.0F);

    // float32_t angle = 0.0F;

    // while (1) {
    //     // as5600_get_angle_data_scaled_bus(&as5600, &angle);
    //     step_motor_set_speed(&motor, 200.0F);

    //     printf("Angle: %f\n\r", angle);
    // }
}