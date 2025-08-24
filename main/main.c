#include "a4988.h"
#include "as5600.h"
#include "gpio.h"
#include "i2c.h"
#include "ina226.h"
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
#define MAX_POSITION (180.0F)
#define MIN_SPEED (1.0F)
#define MAX_SPEED (1000.0F)
#define MIN_ACCELERATION (0.0F)
#define MAX_ACCELERATION (1000.0F)
#define MAGNET_POLARITY (0U)
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

#define REFERENCE_POSITION (0.0F)
#define REFERENCE_SPEED (100.0F)
#define REFERENCE_ACCELERATION (100.0F)

typedef struct {
    TIM_HandleTypeDef* timer;
    uint16_t channel;
} a4988_pwm_user_t;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} a4988_gpio_user_t;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} as5600_gpio_user_t;

typedef struct {
    I2C_HandleTypeDef* bus;
    uint16_t address;
} as5600_i2c_user_t;

typedef struct {
    I2C_HandleTypeDef* bus;
    uint16_t address;
} ina226_i2c_user_t;

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
    a4988_pwm_user_t* pwm_user = (a4988_pwm_user_t*)user;

    HAL_TIM_PWM_Start_IT(pwm_user->timer, pwm_user->channel);

    return A4988_ERR_OK;
}

static a4988_err_t a4988_pwm_stop(void* user)
{
    a4988_pwm_user_t* pwm_user = (a4988_pwm_user_t*)user;

    HAL_TIM_PWM_Stop_IT(pwm_user->timer, pwm_user->channel);

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

        a4988_pwm_user_t* pwm_user = (a4988_pwm_user_t*)user;

        __HAL_TIM_DISABLE(pwm_user->timer);
        __HAL_TIM_SET_COUNTER(pwm_user->timer, 0U);
        __HAL_TIM_SET_PRESCALER(pwm_user->timer, prescaler);
        __HAL_TIM_SET_AUTORELOAD(pwm_user->timer, period);
        __HAL_TIM_SET_COMPARE(pwm_user->timer, pwm_user->channel, compare);
        __HAL_TIM_ENABLE(pwm_user->timer);
    }

    return A4988_ERR_OK;
}

static a4988_err_t a4988_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    a4988_gpio_user_t* gpio_user = (a4988_gpio_user_t*)user;

    HAL_GPIO_WritePin(gpio_user->port,
                      (uint16_t)gpio_user->pin,
                      (GPIO_PinState)!state);

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
    as5600_i2c_user_t* i2c_user = (as5600_i2c_user_t*)user;

    HAL_I2C_Mem_Write(i2c_user->bus,
                      i2c_user->address,
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
    as5600_i2c_user_t* i2c_user = (as5600_i2c_user_t*)user;

    HAL_I2C_Mem_Read(i2c_user->bus,
                     i2c_user->address,
                     address,
                     1U,
                     data,
                     data_size,
                     100);

    return AS5600_ERR_OK;
}

static as5600_err_t as5600_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    as5600_gpio_user_t* gpio_user = (as5600_gpio_user_t*)user;

    HAL_GPIO_WritePin(gpio_user->port,
                      (uint16_t)gpio_user->pin,
                      (GPIO_PinState)state);

    return AS5600_ERR_OK;
}

as5600_err_t as5600_initialize_chip(as5600_t* as5600,
                                    float32_t min_angle,
                                    float32_t max_angle,
                                    bool magnet_polarity)
{
    as5600_status_reg_t status;
    as5600_err_t err = as5600_get_status_reg(as5600, &status);
    if (err != AS5600_ERR_OK) {
        return err;
    }

    float32_t angle_range = (max_angle - min_angle);

    uint16_t min_raw = (uint16_t)(min_angle / angle_range * 4095.0F);
    uint16_t max_raw = (uint16_t)(max_angle / angle_range * 4095.0F);

    as5600_zpos_reg_t zpos = {.zpos = min_raw & 0x0FFF};
    err = as5600_set_zpos_reg(as5600, &zpos);
    if (err != AS5600_ERR_OK) {
        return err;
    }

    as5600_mpos_reg_t mpos = {.mpos = max_raw & 0x0FFF};
    err = as5600_set_mpos_reg(as5600, &mpos);
    if (err != AS5600_ERR_OK) {
        return err;
    }

    as5600_conf_reg_t conf = {.wd = AS5600_WATCHDOG_OFF,
                              .fth = AS5600_SLOW_FILTER_X16,
                              .sf = AS5600_SLOW_FILTER_X16,
                              .pwmf = AS5600_PWM_FREQUENCY_115HZ,
                              .outs = AS5600_FAST_FILTER_THRESH_SLOW,
                              .hyst = AS5600_HYSTERESIS_OFF,
                              .pm = AS5600_POWER_MODE_NOM};
    err = as5600_set_conf_reg(as5600, &conf);
    if (err != AS5600_ERR_OK) {
        return err;
    }

    as5600_zmco_reg_t zmco;
    err = as5600_get_zmco_reg(as5600, &zmco);
    if (err != AS5600_ERR_OK) {
        return err;
    }

    return as5600_set_direction(as5600, (as5600_direction_t)magnet_polarity);
}

motor_driver_err_t motor_driver_encoder_get_position(void* user,
                                                     float32_t* position)
{
    as5600_t* as5600 = (as5600_t*)user;

    as5600_get_angle_data_scaled_bus(as5600, position);

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_fault_get_current(void* user,
                                                  float32_t* current)
{
    ina226_t* ina226 = (ina226_t*)user;

    *current = 1.0F;

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_motor_set_speed(void* user, float32_t speed)
{
    step_motor_t* motor = (step_motor_t*)user;

    step_motor_set_speed(motor, speed);

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_regulator_get_control(void* user,
                                                      float32_t error,
                                                      float32_t* control,
                                                      float32_t delta_time)
{
    pid_regulator_t* regulator = (pid_regulator_t*)user;

    pid_regulator_get_sat_control(regulator, error, delta_time, control);

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

    a4988_pwm_user_t a4988_pwm_user = {.timer = A4988_PWM_TIMER,
                                       .channel = A4988_PWM_CHANNEL};

    a4988_gpio_user_t a4988_gpio_user = {.port = A4988_DIR_GPIO,
                                         .pin = A4988_DIR_PIN};

    as5600_gpio_user_t as5600_gpio_user = {.port = AS5600_DIR_GPIO,
                                           .pin = AS5600_DIR_PIN};

    as5600_i2c_user_t as5600_i2c_user = {.bus = AS5600_I2C_BUS,
                                         .address = AS5600_I2C_ADDRESS};

    ina226_i2c_user_t ina226_i2c_user = {
        .bus = AS5600_I2C_BUS,
        .address = INA226_SLAVE_ADDRESS_A1_GND_A0_GND};

    a4988_t a4988;
    a4988_config_t a4988_config = {};
    a4988_interface_t a4988_interface = {.gpio_user = &a4988_gpio_user,
                                         .gpio_write_pin = a4988_gpio_write_pin,
                                         .pwm_user = &a4988_pwm_user,
                                         .pwm_start = a4988_pwm_start,
                                         .pwm_stop = a4988_pwm_stop,
                                         .pwm_set_frequency =
                                             a4988_pwm_set_frequency};
    a4988_initialize(&a4988, &a4988_config, &a4988_interface);

    as5600_t as5600;
    as5600_config_t as5600_config = {.min_angle = MIN_POSITION,
                                     .max_angle = MAX_POSITION};
    as5600_interface_t as5600_interface = {
        .gpio_user = &as5600_gpio_user,
        .gpio_write_pin = as5600_gpio_write_pin,
        .bus_user = &as5600_i2c_user,
        .bus_write_data = as5600_bus_write_data,
        .bus_read_data = as5600_bus_read_data};
    as5600_initialize(&as5600, &as5600_config, &as5600_interface);

    as5600_initialize_chip(&as5600,
                           MIN_POSITION,
                           MAX_POSITION,
                           MAGNET_POLARITY);

    ina226_t ina226;
    ina226_config_t ina226_config = {};
    ina226_interface_t ina226_interface = {};
    ina226_initialize(&ina226, &ina226_config, &ina226_interface);

    step_motor_t motor;
    step_motor_config_t motor_config = {.min_speed = MIN_SPEED,
                                        .max_speed = MAX_SPEED,
                                        .step_change = STEP_CHANGE,
                                        .should_wrap_position = false};
    step_motor_interface_t motor_interface = {
        .device_user = &a4988,
        .device_set_frequency = step_motor_device_set_frequency,
        .device_set_direction = step_motor_device_set_direction};
    step_motor_initialize(&motor, &motor_config, &motor_interface, 0.0F);

    pid_regulator_t regulator;
    pid_regulator_config_t regulator_config = {.prop_gain = PROP_GAIN,
                                               .int_gain = INT_GAIN,
                                               .dot_gain = DOT_GAIN,
                                               .min_control = MIN_SPEED,
                                               .max_control = MAX_SPEED,
                                               .sat_gain = SAT_GAIN,
                                               .dot_time = DOT_TIME,
                                               .dead_error = DEAD_ERROR};
    pid_regulator_initialize(&regulator, &regulator_config);

    motor_driver_t driver;
    motor_driver_config_t driver_config = {.min_speed = MIN_SPEED,
                                           .max_speed = MAX_SPEED,
                                           .min_position = MIN_POSITION,
                                           .max_position = MAX_POSITION,
                                           .min_acceleration = MIN_ACCELERATION,
                                           .max_acceleration = MAX_ACCELERATION,
                                           .max_current = CURRENT_LIMIT,
                                           .should_wrap_position = false};
    motor_driver_interface_t driver_interface = {
        .encoder_user = &as5600,
        .encoder_get_position = motor_driver_encoder_get_position,
        .fault_user = &ina226,
        .fault_get_current = motor_driver_fault_get_current,
        .motor_user = &motor,
        .motor_set_speed = motor_driver_motor_set_speed,
        .regulator_user = &regulator,
        .regulator_get_control = motor_driver_regulator_get_control};
    motor_driver_initialize(&driver, &driver_config, &driver_interface);

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