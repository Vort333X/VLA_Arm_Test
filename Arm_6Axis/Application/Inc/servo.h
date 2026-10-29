#ifndef SERVO_H
#define SERVO_H

#include "tim.h"
#include "servo_config.h"

typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t pwm_channel;
    uint32_t adc_channel;
    uint16_t adc_at_min;
    uint16_t adc_at_max;

    uint16_t feedback_adc;
    float feedback_deg;
    float target_deg;
    float command_deg;       /* 实际用于换算 PWM 脉宽的指令角度 */
    float error_deg;
    uint16_t pwm_ticks;
    uint8_t enabled;
    uint8_t closed_loop;
    uint8_t feedback_ready;
} Servo_t;

void Servo_Init(Servo_t *servo);
void Servo_SetTarget(Servo_t *servo, float degrees);
void Servo_Enable(Servo_t *servo, uint8_t enable);
void Servo_SetClosedLoop(Servo_t *servo, uint8_t enable);
HAL_StatusTypeDef Servo_UpdateFeedback(Servo_t *servo);
void Servo_Control(Servo_t *servo);
float Servo_AdcToAngle(const Servo_t *servo, uint16_t adc);
uint16_t Servo_AngleToTicks(float degrees);

#endif
