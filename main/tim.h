#ifndef MAIN_TIM_H
#define MAIN_TIM_H

#include "main.h"

extern TIM_HandleTypeDef TIM2_Handle;

void TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* TIM_Handle);

#endif // MAIN_TIM_H
