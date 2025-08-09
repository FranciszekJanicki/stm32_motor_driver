#ifndef MAIN_MAIN_H
#define MAIN_MAIN_H

#include "stm32l476xx.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"

#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

void Error_Handler(void);

#endif // MAIN_MAIN_H
