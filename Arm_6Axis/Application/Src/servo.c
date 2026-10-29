#include "servo.h"
#include "adc_io.h"
#include "pwm_io.h"

static float Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

float Servo_AdcToAngle(const Servo_t *servo, uint16_t adc)
{
    float ratio = ((float)adc - servo->adc_at_min) /
                  ((float)servo->adc_at_max - servo->adc_at_min);
    return Clamp(SERVO_ANGLE_MIN_DEG + ratio *
                 (SERVO_ANGLE_MAX_DEG - SERVO_ANGLE_MIN_DEG),
                 SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
}

uint16_t Servo_AngleToTicks(float degrees)
{
    float ratio = (Clamp(degrees, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG) -
                   SERVO_ANGLE_MIN_DEG) / (SERVO_ANGLE_MAX_DEG - SERVO_ANGLE_MIN_DEG);
    float pulse_us = SERVO_PWM_MIN_US + ratio * (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US);
    return (uint16_t)(pulse_us * SERVO_PWM_TICKS_PER_US + 0.5f);
}

void Servo_Init(Servo_t *servo)
{
    servo->enabled = 0U;
    servo->closed_loop = SERVO_CLOSED_LOOP_DEFAULT;
    servo->feedback_ready = 0U;
    servo->feedback_adc = 0U;
    servo->feedback_deg = 0.0f;
    servo->error_deg = 0.0f;
    servo->target_deg = SERVO_INITIAL_TARGET_DEG;
    servo->command_deg = servo->target_deg;
    servo->pwm_ticks = Servo_AngleToTicks(servo->command_deg);
    PWM_Write(servo->timer, servo->pwm_channel, servo->pwm_ticks);
    Servo_UpdateFeedback(servo);
}

void Servo_SetTarget(Servo_t *servo, float degrees)
{
    /* 重复设置相同目标时保留已有修正量 */
    float target = Clamp(degrees, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    servo->command_deg = Clamp(servo->command_deg + target - servo->target_deg,
                               SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    servo->target_deg = target;
}

void Servo_Enable(Servo_t *servo, uint8_t enable)
{
    servo->enabled = (enable != 0U);
    servo->command_deg = servo->target_deg;
    servo->pwm_ticks = Servo_AngleToTicks(servo->command_deg);
    PWM_Write(servo->timer, servo->pwm_channel, servo->pwm_ticks);
    PWM_Enable(servo->timer, servo->pwm_channel, servo->enabled);
}

void Servo_SetClosedLoop(Servo_t *servo, uint8_t enable)
{
    servo->closed_loop = (enable != 0U);
    servo->command_deg = servo->target_deg;
}

HAL_StatusTypeDef Servo_UpdateFeedback(Servo_t *servo)
{
    uint16_t adc;
    HAL_StatusTypeDef status = ADC_Read(servo->adc_channel, &adc);
    if (status != HAL_OK) return status;

    servo->feedback_adc = adc;
    float angle = Servo_AdcToAngle(servo, adc);
    if (!servo->feedback_ready) servo->feedback_deg = angle;
    else servo->feedback_deg += SERVO_FEEDBACK_FILTER_ALPHA * (angle - servo->feedback_deg);
    servo->feedback_ready = 1U;
    return HAL_OK;
}

void Servo_Control(Servo_t *servo)
{
    servo->error_deg = servo->target_deg - servo->feedback_deg;
    if (!servo->enabled) return;

    if (!servo->closed_loop) {
        servo->command_deg = servo->target_deg;
    } else if (servo->feedback_ready &&
               (servo->error_deg > SERVO_DEADBAND_DEG || servo->error_deg < -SERVO_DEADBAND_DEG)) {
        /* 在舵机内置位置环外进行缓慢的增量修正
         * 误差进入死区后保持上次脉宽，减少抖动 */
        servo->command_deg += SERVO_CORRECTION_GAIN * servo->error_deg;
        servo->command_deg = Clamp(servo->command_deg,
                                   servo->target_deg - SERVO_CORRECTION_LIMIT_DEG,
                                   servo->target_deg + SERVO_CORRECTION_LIMIT_DEG);
    }
    servo->command_deg = Clamp(servo->command_deg, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    servo->pwm_ticks = Servo_AngleToTicks(servo->command_deg);
    PWM_Write(servo->timer, servo->pwm_channel, servo->pwm_ticks);
}
