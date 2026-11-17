#ifndef ARM_H
#define ARM_H

#include "servo.h"

#define ARM_AXIS_COUNT   6U
#define ARM_SERVO_COUNT  7U

typedef enum {
    ARM_SERVO_J1 = 0,
    ARM_SERVO_J2,
    ARM_SERVO_J3,
    ARM_SERVO_J4,
    ARM_SERVO_J5,
    ARM_SERVO_J6,
    ARM_SERVO_J7
} ArmServoId_t;

extern Servo_t servo_j1;
extern Servo_t servo_j2;
extern Servo_t servo_j3;
extern Servo_t servo_j4;
extern Servo_t servo_j5;
extern Servo_t servo_j6;
extern Servo_t servo_j7;

Servo_t *Arm_GetServo(ArmServoId_t id);
void Arm_Init(void);
void Arm_Update(void);       /* 在主循环中每隔 SERVO_CONTROL_PERIOD_MS 毫秒调用 */
void Arm_Enable(uint8_t enable); /* 控制 J1 到 J6，J7 需单独使能 */
void Arm_SetTargets(const float degrees[ARM_AXIS_COUNT]);
void Arm_ResetToCenter(void);  /* 设置 J1 到 J6 的中位目标并使能 PWM */

#endif
