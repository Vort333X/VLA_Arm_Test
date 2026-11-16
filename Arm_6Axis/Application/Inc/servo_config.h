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

/* 依次填写角度 A、ADC A、角度 B、ADC B，两点的 ADC 不能相同 */
#define SERVO_ADC_CALIBRATION(deg_a, raw_a, deg_b, raw_b) \
    { .angle_a_deg = (deg_a), .angle_b_deg = (deg_b), .adc_a = (raw_a), .adc_b = (raw_b) }

/* 先用占位值，装配后在安全行程内选两点实测，H10 暂按 ADC 随角度增大而降低 */
#define SERVO_H3_ADC_CALIBRATION     SERVO_ADC_CALIBRATION(0.0f, 0U,    270.0f, 4095U)
#define SERVO_H7_ADC_CALIBRATION     SERVO_ADC_CALIBRATION(0.0f, 0U,    270.0f, 4095U)
#define SERVO_H5_ADC_CALIBRATION     SERVO_ADC_CALIBRATION(0.0f, 0U,    270.0f, 4095U)
#define SERVO_H10_ADC_CALIBRATION    SERVO_ADC_CALIBRATION(0.0f, 4095U, 270.0f, 0U)
#define SERVO_H8_ADC_CALIBRATION     SERVO_ADC_CALIBRATION(0.0f, 0U,    270.0f, 4095U)
#define SERVO_H6_ADC_CALIBRATION     SERVO_ADC_CALIBRATION(0.0f, 0U,    270.0f, 4095U)
#define SERVO_H4_ADC_CALIBRATION     SERVO_ADC_CALIBRATION(0.0f, 0U,    270.0f, 4095U)

#define SERVO_DEADBAND_DEG           1.0f
#define SERVO_FEEDBACK_FILTER_ALPHA  0.25f
#define SERVO_CORRECTION_GAIN        0.10f
#define SERVO_CORRECTION_LIMIT_DEG   15.0f
#define SERVO_CLOSED_LOOP_DEFAULT    0U  /* ADC 反馈标定后再使能外环 */

#define ADC_SAMPLE_TIME          ADC_SAMPLETIME_71CYCLES_5
#define ADC_TIMEOUT_MS           1U

#endif
