#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

/* PWM 参数与 IOC 一致：TIM1/TIM2 时钟为 48 MHz，PSC 为 15，ARR 为 59999 */
#define SERVO_PWM_TICKS_PER_US       3.0f
#define SERVO_PWM_MIN_US             500.0f
#define SERVO_PWM_MAX_US             2500.0f
#define SERVO_ANGLE_MIN_DEG          0.0f
#define SERVO_ANGLE_MAX_DEG          270.0f
#define SERVO_INITIAL_TARGET_DEG     135.0f
#define SERVO_CONTROL_PERIOD_MS      20U

/* 以下为占位标定值，需实测每路在 0 度和 270 度时的 ADC 值
 * 若 ADC 电压随角度增大而降低，可将两个端点设为递减值 */
#define SERVO_H3_ADC_AT_MIN          0U
#define SERVO_H3_ADC_AT_MAX          4095U
#define SERVO_H5_ADC_AT_MIN          0U
#define SERVO_H5_ADC_AT_MAX          4095U
#define SERVO_H4_ADC_AT_MIN          0U
#define SERVO_H4_ADC_AT_MAX          4095U
#define SERVO_H6_ADC_AT_MIN          0U
#define SERVO_H6_ADC_AT_MAX          4095U
#define SERVO_H8_ADC_AT_MIN          0U
#define SERVO_H8_ADC_AT_MAX          4095U
#define SERVO_H7_ADC_AT_MIN          0U
#define SERVO_H7_ADC_AT_MAX          4095U
#define SERVO_H10_ADC_AT_MIN         0U
#define SERVO_H10_ADC_AT_MAX         4095U

#define SERVO_DEADBAND_DEG           1.0f
#define SERVO_FEEDBACK_FILTER_ALPHA  0.25f
#define SERVO_CORRECTION_GAIN        0.10f
#define SERVO_CORRECTION_LIMIT_DEG   15.0f
#define SERVO_CLOSED_LOOP_DEFAULT    1U

#define ADC_SAMPLE_TIME          ADC_SAMPLETIME_71CYCLES_5
#define ADC_TIMEOUT_MS           1U

#endif
