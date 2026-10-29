#include "arm.h"
#include "adc.h"

/* 轴序依次为三个 TIM1 插座及 H6、H8、H7，H10 为备用路 */
Servo_t arm_servos[ARM_SERVO_COUNT] = {
    {.timer = &htim1, .pwm_channel = TIM_CHANNEL_1, .adc_channel = ADC_CHANNEL_0,
     .adc_at_min = SERVO_H3_ADC_AT_MIN, .adc_at_max = SERVO_H3_ADC_AT_MAX},
    {.timer = &htim1, .pwm_channel = TIM_CHANNEL_2, .adc_channel = ADC_CHANNEL_1,
     .adc_at_min = SERVO_H5_ADC_AT_MIN, .adc_at_max = SERVO_H5_ADC_AT_MAX},
    {.timer = &htim1, .pwm_channel = TIM_CHANNEL_3, .adc_channel = ADC_CHANNEL_2,
     .adc_at_min = SERVO_H4_ADC_AT_MIN, .adc_at_max = SERVO_H4_ADC_AT_MAX},
    {.timer = &htim2, .pwm_channel = TIM_CHANNEL_2, .adc_channel = ADC_CHANNEL_4,
     .adc_at_min = SERVO_H6_ADC_AT_MIN, .adc_at_max = SERVO_H6_ADC_AT_MAX},
    {.timer = &htim2, .pwm_channel = TIM_CHANNEL_3, .adc_channel = ADC_CHANNEL_5,
     .adc_at_min = SERVO_H8_ADC_AT_MIN, .adc_at_max = SERVO_H8_ADC_AT_MAX},
    {.timer = &htim2, .pwm_channel = TIM_CHANNEL_4, .adc_channel = ADC_CHANNEL_6,
     .adc_at_min = SERVO_H7_ADC_AT_MIN, .adc_at_max = SERVO_H7_ADC_AT_MAX},
    {.timer = &htim2, .pwm_channel = TIM_CHANNEL_1, .adc_channel = ADC_CHANNEL_3,
     .adc_at_min = SERVO_H10_ADC_AT_MIN, .adc_at_max = SERVO_H10_ADC_AT_MAX},
};

void Arm_Init(void)
{
    for (uint32_t i = 0; i < ARM_SERVO_COUNT; ++i) Servo_Init(&arm_servos[i]);
}

void Arm_Update(void)
{
    for (uint32_t i = 0; i < ARM_SERVO_COUNT; ++i) {
        if (Servo_UpdateFeedback(&arm_servos[i]) == HAL_OK) Servo_Control(&arm_servos[i]);
    }
}

void Arm_Enable(uint8_t enable)
{
    for (uint32_t i = 0; i < ARM_AXIS_COUNT; ++i) Servo_Enable(&arm_servos[i], enable);
}

void Arm_SetTargets(const float degrees[ARM_AXIS_COUNT])
{
    for (uint32_t i = 0; i < ARM_AXIS_COUNT; ++i) Servo_SetTarget(&arm_servos[i], degrees[i]);
}
