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

#define VAL_LIMIT(value, low, high) \
    (((value) < (low)) ? (low) : (((value) > (high)) ? (high) : (value)))

/* 各路输出限位用舵机命令角填写，实测后收窄，PWM 脉宽映射仍按全量程 */
#define SERVO_OUTPUT_LIMITS(low, high) { .min_deg = (low), .max_deg = (high) }
#define SERVO_H3_OUTPUT_LIMITS       SERVO_OUTPUT_LIMITS(0.0f, 270.0f)
#define SERVO_H7_OUTPUT_LIMITS       SERVO_OUTPUT_LIMITS(0.0f, 270.0f)
#define SERVO_H5_OUTPUT_LIMITS       SERVO_OUTPUT_LIMITS(0.0f, 270.0f)
#define SERVO_H10_OUTPUT_LIMITS      SERVO_OUTPUT_LIMITS(0.0f, 270.0f)
#define SERVO_H8_OUTPUT_LIMITS       SERVO_OUTPUT_LIMITS(0.0f, 270.0f)
#define SERVO_H6_OUTPUT_LIMITS       SERVO_OUTPUT_LIMITS(0.0f, 270.0f)
#define SERVO_H4_OUTPUT_LIMITS       SERVO_OUTPUT_LIMITS(0.0f, 270.0f)

/* 依次填写角度 A、ADC A、角度 B、ADC B，两点的 ADC 不能相同 */
#define SERVO_ADC_CALIBRATION(deg_a, raw_a, deg_b, raw_b) \
    { .angle_a_deg = (deg_a), .angle_b_deg = (deg_b), .adc_a = (raw_a), .adc_b = (raw_b) }

/* 未标定时只选命令角增大对应的 ADC 方向，暂按 12 位满量程换算 */
#define SERVO_ADC_INCREASING         0U
#define SERVO_ADC_DECREASING         1U
#define SERVO_ADC_RAW_MAX            4095U
#define SERVO_ADC_UNCALIBRATED(direction) \
    SERVO_ADC_CALIBRATION(SERVO_ANGLE_MIN_DEG, \
        ((direction) == SERVO_ADC_DECREASING ? SERVO_ADC_RAW_MAX : 0U), \
        SERVO_ANGLE_MAX_DEG, \
        ((direction) == SERVO_ADC_DECREASING ? 0U : SERVO_ADC_RAW_MAX))

/* 实测两点后把对应行换成 SERVO_ADC_CALIBRATION */
#define SERVO_H3_ADC_CALIBRATION     SERVO_ADC_UNCALIBRATED(SERVO_ADC_INCREASING)
#define SERVO_H7_ADC_CALIBRATION     SERVO_ADC_UNCALIBRATED(SERVO_ADC_INCREASING)
#define SERVO_H5_ADC_CALIBRATION     SERVO_ADC_UNCALIBRATED(SERVO_ADC_INCREASING)
#define SERVO_H10_ADC_CALIBRATION    SERVO_ADC_UNCALIBRATED(SERVO_ADC_DECREASING)
#define SERVO_H8_ADC_CALIBRATION     SERVO_ADC_UNCALIBRATED(SERVO_ADC_INCREASING)
#define SERVO_H6_ADC_CALIBRATION     SERVO_ADC_UNCALIBRATED(SERVO_ADC_INCREASING)
#define SERVO_H4_ADC_CALIBRATION     SERVO_ADC_UNCALIBRATED(SERVO_ADC_INCREASING)

#define SERVO_DEADBAND_DEG           1.0f
#define SERVO_FEEDBACK_FILTER_ALPHA  0.25f
#define SERVO_CORRECTION_GAIN        0.10f
#define SERVO_CORRECTION_LIMIT_DEG   15.0f
#define SERVO_CLOSED_LOOP_DEFAULT    0U  /* ADC 反馈标定后再使能外环 */

#define ADC_SAMPLE_TIME          ADC_SAMPLETIME_71CYCLES_5
#define ADC_TIMEOUT_MS           1U

#endif
