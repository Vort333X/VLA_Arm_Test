#ifndef ARM_H
#define ARM_H

#include "servo.h"

#define ARM_AXIS_COUNT   6U
#define ARM_SERVO_COUNT  7U

typedef enum {
    ARM_SERVO_H3 = 0,
    ARM_SERVO_H5,
    ARM_SERVO_H4,
    ARM_SERVO_H6,
    ARM_SERVO_H8,
    ARM_SERVO_H7,
    ARM_SERVO_H10_SPARE
} ArmServoId_t;

extern Servo_t arm_servos[ARM_SERVO_COUNT];

void Arm_Init(void);
void Arm_Update(void);       /* 在主循环中每隔 SERVO_CONTROL_PERIOD_MS 毫秒调用 */
void Arm_Enable(uint8_t enable); /* 仅控制六轴，备用路需单独使能 */
void Arm_SetTargets(const float degrees[ARM_AXIS_COUNT]);

#endif
