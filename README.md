# Servo Arm

五轴机械臂与夹爪，由 STM32F103C8T6 裸机控制，通过 USART1 V2 与上位机通信。

- [坐标设置与参考姿态](WebHost/README.md)：坐标图、关节零位、模型参数和角度换算。
- [下位机软件说明](Servo_Arm/README.md)：控制架构、PWM 与逐轴限幅、USART1 V2 帧格式。
