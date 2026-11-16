#include "arm.h"
#include "adc.h"

/* 独立的全局结构体便于通过 SWD 观察每路舵机 */
Servo_t servo_h3 = {
    .timer = &htim1, .pwm_channel = TIM_CHANNEL_1, .adc_channel = ADC_CHANNEL_0,
    .feedback_cal = SERVO_H3_ADC_CALIBRATION,
};
Servo_t servo_h5 = {
    .timer = &htim1, .pwm_channel = TIM_CHANNEL_2, .adc_channel = ADC_CHANNEL_1,
    .feedback_cal = SERVO_H5_ADC_CALIBRATION,
};
Servo_t servo_h4 = {
    .timer = &htim1, .pwm_channel = TIM_CHANNEL_3, .adc_channel = ADC_CHANNEL_2,
    .feedback_cal = SERVO_H4_ADC_CALIBRATION,
};
Servo_t servo_h6 = {
    .timer = &htim2, .pwm_channel = TIM_CHANNEL_2, .adc_channel = ADC_CHANNEL_4,
    .feedback_cal = SERVO_H6_ADC_CALIBRATION,
};
Servo_t servo_h8 = {
    .timer = &htim2, .pwm_channel = TIM_CHANNEL_3, .adc_channel = ADC_CHANNEL_5,
    .feedback_cal = SERVO_H8_ADC_CALIBRATION,
};
Servo_t servo_h7 = {
    .timer = &htim2, .pwm_channel = TIM_CHANNEL_4, .adc_channel = ADC_CHANNEL_6,
    .feedback_cal = SERVO_H7_ADC_CALIBRATION,
};
Servo_t servo_h10 = {
    .timer = &htim2, .pwm_channel = TIM_CHANNEL_1, .adc_channel = ADC_CHANNEL_3,
    .feedback_cal = SERVO_H10_ADC_CALIBRATION,
};

/* 六轴按底座到末端排列，H4 放在备用位 */
static Servo_t *const servo_by_id[ARM_SERVO_COUNT] = {
    [ARM_SERVO_H3] = &servo_h3,
    [ARM_SERVO_H7] = &servo_h7,
    [ARM_SERVO_H5] = &servo_h5,
    [ARM_SERVO_H10] = &servo_h10,
    [ARM_SERVO_H8] = &servo_h8,
    [ARM_SERVO_H6] = &servo_h6,
    [ARM_SERVO_H4_SPARE] = &servo_h4,
};

Servo_t *Arm_GetServo(ArmServoId_t id)
{
    return servo_by_id[id];
}

void Arm_Init(void)
{
    for (uint32_t i = 0; i < ARM_SERVO_COUNT; ++i) Servo_Init(servo_by_id[i]);
}

void Arm_Update(void)
{
    for (uint32_t i = 0; i < ARM_SERVO_COUNT; ++i) {
        Servo_t *servo = servo_by_id[i];
        if (Servo_UpdateFeedback(servo) == HAL_OK) Servo_Control(servo);
    }
}

void Arm_Enable(uint8_t enable)
{
    for (uint32_t i = 0; i < ARM_AXIS_COUNT; ++i) Servo_Enable(servo_by_id[i], enable);
}

void Arm_SetTargets(const float degrees[ARM_AXIS_COUNT])
{
    for (uint32_t i = 0; i < ARM_AXIS_COUNT; ++i) Servo_SetTarget(servo_by_id[i], degrees[i]);
}

void Arm_ResetToCenter(void)
{
    const float center = (SERVO_ANGLE_MIN_DEG + SERVO_ANGLE_MAX_DEG) * 0.5f;
    for (uint32_t i = 0; i < ARM_AXIS_COUNT; ++i) Servo_SetTarget(servo_by_id[i], center);
    Arm_Enable(1U);
}
