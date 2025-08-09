#include "i2c.h"

I2C_HandleTypeDef I2C1_Handle = {0};

void I2C1_Init(void)
{
    I2C1_Handle.Instance = I2C1;
    I2C1_Handle.Init.Timing = 0x10D19CE4;
    I2C1_Handle.Init.OwnAddress1 = 0;
    I2C1_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2C1_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2C1_Handle.Init.OwnAddress2 = 0;
    I2C1_Handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    I2C1_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2C1_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&I2C1_Handle) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&I2C1_Handle, I2C_ANALOGFILTER_ENABLE) !=
        HAL_OK) {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&I2C1_Handle, 0) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* I2C_Handle)
{
    GPIO_InitTypeDef GPIO_Init = {0};
    RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInit = {0};

    if (I2C_Handle->Instance == I2C1) {
        RCC_PeriphCLKInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
        RCC_PeriphCLKInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;

        if (HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInit) != HAL_OK) {
            Error_Handler();
        }

        GPIO_Init.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_Init.Mode = GPIO_MODE_AF_OD;
        GPIO_Init.Pull = GPIO_NOPULL;
        GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_Init.Alternate = GPIO_AF4_I2C1;

        HAL_GPIO_Init(GPIOB, &GPIO_Init);

        __HAL_RCC_I2C1_CLK_ENABLE();
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* I2C_Handle)
{
    if (I2C_Handle->Instance == I2C1) {
        __HAL_RCC_I2C1_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    }
}
