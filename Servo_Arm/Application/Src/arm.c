#include "arm.h"
#include "adc.h"

/* 独立的全局结构体便于通过 SWD 观察每路舵机 */
Servo_t servo_j1 = {
    .timer = &htim1, .pwm_channel = TIM_CHANNEL_1, .adc_channel = ADC_CHANNEL_0,
    .feedback_cal = SERVO_J1_ADC_CALIBRATION,
    .output_limits = SERVO_J1_OUTPUT_LIMITS,
};
Servo_t servo_j2 = {
    .timer = &htim2, .pwm_channel = TIM_CHANNEL_4, .adc_channel = ADC_CHANNEL_6,
    .feedback_cal = SERVO_J2_ADC_CALIBRATION,
    .output_limits = SERVO_J2_OUTPUT_LIMITS,
};
Servo_t servo_j3 = {
    .timer = &htim1, .pwm_channel = TIM_CHANNEL_2, .adc_channel = ADC_CHANNEL_1,
    .feedback_cal = SERVO_J3_ADC_CALIBRATION,
    .output_limits = SERVO_J3_OUTPUT_LIMITS,
};
Servo_t servo_j4 = {
    .timer = &htim2, .pwm_channel = TIM_CHANNEL_1, .adc_channel = ADC_CHANNEL_3,
    .feedback_cal = SERVO_J4_ADC_CALIBRATION,
    .output_limits = SERVO_J4_OUTPUT_LIMITS,
};
Servo_t servo_j5 = {
    .timer = &htim2, .pwm_channel = TIM_CHANNEL_3, .adc_channel = ADC_CHANNEL_5,
    .feedback_cal = SERVO_J5_ADC_CALIBRATION,
    .output_limits = SERVO_J5_OUTPUT_LIMITS,
};
Servo_t servo_j6 = {
    .timer = &htim2, .pwm_channel = TIM_CHANNEL_2, .adc_channel = ADC_CHANNEL_4,
    .feedback_cal = SERVO_J6_ADC_CALIBRATION,
    .output_limits = SERVO_J6_OUTPUT_LIMITS,
};
Servo_t servo_j7 = {
    .timer = &htim1, .pwm_channel = TIM_CHANNEL_3, .adc_channel = ADC_CHANNEL_2,
    .feedback_cal = SERVO_J7_ADC_CALIBRATION,
    .output_limits = SERVO_J7_OUTPUT_LIMITS,
};

/* J1 到 J5 从底座到末端排列，J6 是夹爪，J7 备用 */
static Servo_t *const servo_by_id[ARM_SERVO_COUNT] = {
    [ARM_SERVO_J1] = &servo_j1,
    [ARM_SERVO_J2] = &servo_j2,
    [ARM_SERVO_J3] = &servo_j3,
    [ARM_SERVO_J4] = &servo_j4,
    [ARM_SERVO_J5] = &servo_j5,
    [ARM_SERVO_J6] = &servo_j6,
    [ARM_SERVO_J7] = &servo_j7,
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
