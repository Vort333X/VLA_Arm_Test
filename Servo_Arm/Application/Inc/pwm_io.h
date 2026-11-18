#ifndef PWM_IO_H
#define PWM_IO_H

#include "tim.h"

void PWM_Write(TIM_HandleTypeDef *timer, uint32_t channel, uint16_t ticks);
void PWM_Enable(TIM_HandleTypeDef *timer, uint32_t channel, uint8_t enable);

#endif
