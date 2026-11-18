#include "pwm_io.h"

void PWM_Write(TIM_HandleTypeDef *timer, uint32_t channel, uint16_t ticks)
{
    __HAL_TIM_SET_COMPARE(timer, channel, ticks);
}

void PWM_Enable(TIM_HandleTypeDef *timer, uint32_t channel, uint8_t enable)
{
    if (enable) HAL_TIM_PWM_Start(timer, channel);
    else HAL_TIM_PWM_Stop(timer, channel);
}
