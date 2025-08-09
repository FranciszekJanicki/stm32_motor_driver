#include "tim.h"

TIM_HandleTypeDef TIM2_Handle = {0};

void TIM2_Init(void)
{
    TIM_ClockConfigTypeDef TIM_ClockConfig = {0};
    TIM_MasterConfigTypeDef TIM_MasterConfig = {0};
    TIM_OC_InitTypeDef TIM_OC_Init = {0};

    TIM2_Handle.Instance = TIM2;
    TIM2_Handle.Init.Prescaler = 0;
    TIM2_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM2_Handle.Init.Period = 4294967295;
    TIM2_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM2_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&TIM2_Handle) != HAL_OK) {
        Error_Handler();
    }

    TIM_ClockConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&TIM2_Handle, &TIM_ClockConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Init(&TIM2_Handle) != HAL_OK) {
        Error_Handler();
    }

    TIM_MasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    TIM_MasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&TIM2_Handle,
                                              &TIM_MasterConfig) != HAL_OK) {
        Error_Handler();
    }

    TIM_OC_Init.OCMode = TIM_OCMODE_PWM1;
    TIM_OC_Init.Pulse = 0;
    TIM_OC_Init.OCPolarity = TIM_OCPOLARITY_HIGH;
    TIM_OC_Init.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&TIM2_Handle, &TIM_OC_Init, TIM_CHANNEL_1) !=
        HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&TIM2_Handle);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* TIM_Handle)
{
    if (TIM_Handle->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* TIM_Handle)
{
    GPIO_InitTypeDef GPIO_Init = {0};

    if (TIM_Handle->Instance == TIM2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_Init.Pin = GPIO_PIN_0;
        GPIO_Init.Mode = GPIO_MODE_AF_PP;
        GPIO_Init.Pull = GPIO_NOPULL;
        GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_Init.Alternate = GPIO_AF1_TIM2;

        HAL_GPIO_Init(GPIOA, &GPIO_Init);
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* TIM_Handle)
{
    if (TIM_Handle->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_DISABLE();
    }
}
