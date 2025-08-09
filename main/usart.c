#include "usart.h"

UART_HandleTypeDef UART2_Handle = {0};

void UART2_Init(void)
{
    UART2_Handle.Instance = USART2;
    UART2_Handle.Init.BaudRate = 115200;
    UART2_Handle.Init.WordLength = UART_WORDLENGTH_8B;
    UART2_Handle.Init.StopBits = UART_STOPBITS_1;
    UART2_Handle.Init.Parity = UART_PARITY_NONE;
    UART2_Handle.Init.Mode = UART_MODE_TX_RX;
    UART2_Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UART2_Handle.Init.OverSampling = UART_OVERSAMPLING_16;
    UART2_Handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    UART2_Handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&UART2_Handle) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef* UART_Handle)
{
    if (UART_Handle->Instance == USART2) {
        RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInit = {
            .PeriphClockSelection = RCC_PERIPHCLK_USART2,
            .Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1};
        if (HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInit) != HAL_OK) {
            Error_Handler();
        }

        __HAL_RCC_USART2_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_Init = {.Pin = USART_TX_Pin | USART_RX_Pin,
                                      .Mode = GPIO_MODE_AF_PP,
                                      .Pull = GPIO_NOPULL,
                                      .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                                      .Alternate = GPIO_AF7_USART2};
        HAL_GPIO_Init(USART_TX_GPIO_Port, &GPIO_Init);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* UART_Handle)
{
    if (UART_Handle->Instance == USART2) {
        __HAL_RCC_USART2_CLK_DISABLE();

        HAL_GPIO_DeInit(USART_TX_GPIO_Port, USART_TX_Pin | USART_RX_Pin);
    }
}