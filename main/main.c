#include "a4988.h"
#include "as5600.h"
#include "gpio.h"
#include "i2c.h"
#include "motor_driver.h"
#include "pid_regulator.h"
#include "rotary_encoder.h"
#include "step_motor.h"
#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define PROP_GAIN (5.0F)
#define INT_GAIN (0.0F)
#define DOT_GAIN (0.0F)
#define SAT_GAIN (0.0F)
#define DOT_TIME (0.0F)
#define MIN_POSITION (0.0F)
#define MAX_POSITION (359.0F)
#define MIN_SPEED (1.0F)
#define MAX_SPEED (1000.0F)
#define MIN_ACCELERATION (0.0F)
#define MAX_ACCELERATION (1000.0F)
#define STEP_CHANGE (1.8F / 16.0F)
#define CURRENT_LIMIT (2.0F)
#define DEAD_ERROR (1.8F / 16.0F)

#define DELTA_TIME (10.0F / 1000.0F)
#define DELTA_TIMER (&htim1)

#define AS5600_DIR_GPIO GPIOC
#define AS5600_DIR_PIN (1U << 2U)
#define AS5600_I2C_BUS (&hi2c1)
#define AS5600_I2C_ADDRESS (AS5600_SLAVE_ADDRESS << 1U)

#define A4988_DIR_GPIO GPIOA
#define A4988_DIR_PIN (1U << 9U)
#define A4988_PWM_TIMER (&htim2)
#define A4988_PWM_CHANNEL (TIM_CHANNEL_1)

#define REFERENCE_POSITION (300.0F)
#define REFERENCE_SPEED (100.0F)
#define REFERENCE_ACCELERATION (100.0F)

static bool frequency_to_prescaler_and_period(uint32_t frequency_hz,
                                              uint32_t clock_hz,
                                              uint32_t max_prescaler,
                                              uint32_t max_period,
                                              uint32_t* prescaler,
                                              uint32_t* period)
{
    if (frequency_hz == 0U || !prescaler || !period) {
        return false;
    }

    uint32_t temp_prescaler = 0U;
    uint32_t temp_period = clock_hz / frequency_hz;

    while (temp_period > max_period && temp_prescaler < max_prescaler) {
        temp_prescaler++;
        temp_period = clock_hz / ((temp_prescaler + 1U) * frequency_hz);
    }
    if (temp_period > max_period) {
        temp_period = max_period;
        temp_prescaler = (clock_hz / (temp_period * frequency_hz)) - 1U;
    }
    if (temp_prescaler > max_prescaler) {
        temp_prescaler = max_prescaler;
    }

    *prescaler = temp_prescaler;
    *period = temp_period;

    return true;
}

static a4988_err_t a4988_pwm_start(void* user)
{
    HAL_TIM_PWM_Start_IT(A4988_PWM_TIMER, A4988_PWM_CHANNEL);

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_stop(void* user)
{
    HAL_TIM_PWM_Stop_IT(A4988_PWM_TIMER, A4988_PWM_CHANNEL);

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_set_frequency(void* user, uint32_t frequency)
{
    uint32_t clock_hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        clock_hz *= 2;
    }

    uint32_t period;
    uint32_t prescaler;
    bool result = frequency_to_prescaler_and_period(frequency,
                                                    clock_hz,
                                                    0xFFFFU,
                                                    0xFFFFU,
                                                    &prescaler,
                                                    &period);

    if (result && period < 0xFFFFU && prescaler < 0xFFFFU) {
        uint32_t tick_hz = clock_hz / (prescaler + 1);
        uint32_t compare = (tick_hz / 1000000) * 5; // 5 us pulse
        if (compare == 0) {
            compare = 1;
        }
        if (compare > period) {
            compare = period;
        }

        __HAL_TIM_DISABLE(A4988_PWM_TIMER);
        __HAL_TIM_SET_COUNTER(A4988_PWM_TIMER, 0U);
        __HAL_TIM_SET_PRESCALER(A4988_PWM_TIMER, prescaler);
        __HAL_TIM_SET_AUTORELOAD(A4988_PWM_TIMER, period);
        __HAL_TIM_SET_COMPARE(A4988_PWM_TIMER, A4988_PWM_CHANNEL, compare);
        __HAL_TIM_ENABLE(A4988_PWM_TIMER);
    }

    return A4988_ERR_OK;
}

static a4988_err_t a4988_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    HAL_GPIO_WritePin(user, (uint16_t)pin, (GPIO_PinState)!state);

    return A4988_ERR_OK;
}

static step_motor_err_t step_motor_device_set_frequency(void* user,
                                                        uint32_t frequency)
{
    a4988_t* a4988 = (a4988_t*)user;

    a4988_set_frequency(a4988, frequency);

    return STEP_MOTOR_ERR_OK;
}

static step_motor_err_t step_motor_device_set_direction(
    void* user,
    step_motor_direction_t direction)
{
    a4988_t* a4988 = (a4988_t*)user;

    a4988_set_direction(a4988, (a4988_direction_t)direction);

    return STEP_MOTOR_ERR_OK;
}

static as5600_err_t as5600_bus_write_data(void* user,
                                          uint8_t address,
                                          uint8_t const* data,
                                          size_t data_size)
{
    HAL_I2C_Mem_Write(AS5600_I2C_BUS,
                      AS5600_I2C_ADDRESS,
                      address,
                      1U,
                      (uint8_t*)data,
                      data_size,
                      100);

    return AS5600_ERR_OK;
}

static as5600_err_t as5600_bus_read_data(void* user,
                                         uint8_t address,
                                         uint8_t* data,
                                         size_t data_size)
{
    HAL_I2C_Mem_Read(AS5600_I2C_BUS,
                     AS5600_I2C_ADDRESS,
                     address,
                     1U,
                     data,
                     data_size,
                     100);

    return AS5600_ERR_OK;
}

static as5600_err_t as5600_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    HAL_GPIO_WritePin(user, (uint16_t)pin, (GPIO_PinState)state);

    return AS5600_ERR_OK;
}

as5600_err_t as5600_init_chip(as5600_t* as5600,
                              float32_t min_angle,
                              float32_t max_angle)
{
    as5600_status_reg_t status;
    as5600_err_t err = as5600_get_status_reg(as5600, &status);
    if (err != AS5600_ERR_OK)
        return err;

    float32_t angle_range = (max_angle - min_angle);

    uint16_t min_raw = (uint16_t)(min_angle / angle_range * 4095.0F);
    uint16_t max_raw = (uint16_t)(max_angle / angle_range * 4095.0F);

    as5600_zpos_reg_t zpos = {.zpos = min_raw & 0x0FFF};
    err = as5600_set_zpos_reg(as5600, &zpos);
    if (err != AS5600_ERR_OK)
        return err;

    as5600_mpos_reg_t mpos = {.mpos = max_raw & 0x0FFF};
    err = as5600_set_mpos_reg(as5600, &mpos);
    if (err != AS5600_ERR_OK)
        return err;

    as5600_conf_reg_t conf = {.wd = AS5600_WATCHDOG_OFF,
                              .fth = AS5600_SLOW_FILTER_X16,
                              .sf = AS5600_SLOW_FILTER_X16,
                              .pwmf = AS5600_PWM_FREQUENCY_115HZ,
                              .outs = AS5600_FAST_FILTER_THRESH_SLOW,
                              .hyst = AS5600_HYSTERESIS_OFF,
                              .pm = AS5600_POWER_MODE_NOM};
    err = as5600_set_conf_reg(as5600, &conf);
    if (err != AS5600_ERR_OK)
        return err;

    as5600_zmco_reg_t zmco;
    err = as5600_get_zmco_reg(as5600, &zmco);
    if (err != AS5600_ERR_OK)
        return err;

    return AS5600_ERR_OK;
}

motor_driver_err_t motor_driver_encoder_get_position(void* user,
                                                     float32_t* position)
{
    as5600_get_angle_data_scaled_bus(user, position);

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_fault_get_current(void* user,
                                                  float32_t* current)
{
    *current = 1.0F;

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_motor_set_speed(void* user, float32_t speed)
{
    step_motor_set_speed(user, speed);

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_regulator_get_control(void* user,
                                                      float32_t error,
                                                      float32_t* control,
                                                      float32_t delta_time)
{
    pid_regulator_get_sat_control(user, error, delta_time, control);

    return MOTOR_DRIVER_ERR_OK;
}

static void delta_timer_start(void)
{
    HAL_TIM_Base_Start_IT(DELTA_TIMER);
}

static bool volatile has_delta_timer_elapsed = false;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == DELTA_TIMER) {
        has_delta_timer_elapsed = true;
    }
}

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_TIM1_Init();
    MX_I2C1_Init();

    a4988_t a4988;
    a4988_initialize(
        &a4988,
        &(a4988_config_t){.pin_dir = A4988_DIR_PIN},
        &(a4988_interface_t){.gpio_user = A4988_DIR_GPIO,
                             .gpio_write_pin = a4988_gpio_write_pin,
                             .pwm_start = a4988_pwm_start,
                             .pwm_stop = a4988_pwm_stop,
                             .pwm_set_frequency = a4988_pwm_set_frequency});

    as5600_t as5600;
    as5600_initialize(
        &as5600,
        &(as5600_config_t){.dir_pin = AS5600_DIR_PIN,
                           .min_angle = MIN_POSITION,
                           .max_angle = MAX_POSITION},
        &(as5600_interface_t){.gpio_user = AS5600_DIR_GPIO,
                              .gpio_write_pin = as5600_gpio_write_pin,
                              .bus_write_data = as5600_bus_write_data,
                              .bus_read_data = as5600_bus_read_data});

    as5600_init_chip(&as5600, MIN_POSITION, MAX_POSITION);

    step_motor_t motor;
    step_motor_initialize(
        &motor,
        &(step_motor_config_t){.min_speed = MIN_SPEED,
                               .max_speed = MAX_SPEED,
                               .step_change = STEP_CHANGE,
                               .should_wrap_position = false},
        &(step_motor_interface_t){
            .device_user = &a4988,
            .device_set_frequency = step_motor_device_set_frequency,
            .device_set_direction = step_motor_device_set_direction},
        0.0F);

    pid_regulator_t regulator;
    pid_regulator_initialize(
        &regulator,
        &(pid_regulator_config_t){.prop_gain = PROP_GAIN,
                                  .int_gain = INT_GAIN,
                                  .dot_gain = DOT_GAIN,
                                  .min_control = MIN_SPEED,
                                  .max_control = MAX_SPEED,
                                  .sat_gain = SAT_GAIN,
                                  .dot_time = DOT_TIME,
                                  .dead_error = DEAD_ERROR});

    motor_driver_t driver;
    motor_driver_initialize(
        &driver,
        &(motor_driver_config_t){.min_speed = MIN_SPEED,
                                 .max_speed = MAX_SPEED,
                                 .min_position = MIN_POSITION,
                                 .max_position = MAX_POSITION,
                                 .min_acceleration = MIN_ACCELERATION,
                                 .max_acceleration = MAX_ACCELERATION,
                                 .max_current = CURRENT_LIMIT,
                                 .should_wrap_position = false},
        &(motor_driver_interface_t){
            .encoder_user = &as5600,
            .encoder_get_position = motor_driver_encoder_get_position,
            .fault_get_current = motor_driver_fault_get_current,
            .motor_user = &motor,
            .motor_set_speed = motor_driver_motor_set_speed,
            .regulator_user = &regulator,
            .regulator_get_control = motor_driver_regulator_get_control});

    delta_timer_start();

    motor_driver_state_t state;

    while (1) {
        if (has_delta_timer_elapsed) {
            motor_driver_set_position(&driver, REFERENCE_POSITION, DELTA_TIME);
            motor_driver_get_state(&driver, &state);
            printf("POS: %f, REF: %f, ERR: %f, SPD: %f, CUR: %f\n\r",
                   state.measure_position,
                   REFERENCE_POSITION,
                   REFERENCE_POSITION - state.measure_position,
                   state.control_speed,
                   state.fault_current);

            has_delta_timer_elapsed = false;
        }
    }
}