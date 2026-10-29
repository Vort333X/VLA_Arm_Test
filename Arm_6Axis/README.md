# 六轴舵机机械臂底层

STM32F103C8T6，HAL 裸机。六轴 + 一路备用，USB CDC 保留。无 RTOS、运动学或上位机解析器。

## 接口与轴序

每个舵机插座：1 GND、2 舵机电源、3 PWM、4 ADC 反馈。

| 实例 / 数组下标 | 插座 | PWM | MCU 引脚 | ADC1 通道 / 引脚 |
|---|---|---|---|---|
| ARM_SERVO_H3 / 0 | H3 | TIM1_CH1 | PA8 | IN0 / PA0 |
| ARM_SERVO_H5 / 1 | H5 | TIM1_CH2 | PA9 | IN1 / PA1 |
| ARM_SERVO_H4 / 2 | H4 | TIM1_CH3 | PA10 | IN2 / PA2 |
| ARM_SERVO_H6 / 3 | H6 | TIM2_CH2 | PB3 | IN4 / PA4 |
| ARM_SERVO_H8 / 4 | H8 | TIM2_CH3 | PB10 | IN5 / PA5 |
| ARM_SERVO_H7 / 5 | H7 | TIM2_CH4 | PB11 | IN6 / PA6 |
| ARM_SERVO_H10_SPARE / 6 | H10 | TIM2_CH1 | PA15 | IN3 / PA3 |

H9：PB6 = USART1_TX，PB7 = USART1_RX，115200 / 8N1。USB：PA11 DM、PA12 DP。
TIM2 全重映射、USART1 重映射；禁用 JTAG 释放 PA15/PB3，保留 SWD。
上述轴序只是软件默认，可在 `arm.c` 的实例表里调整。

## IOC 与 PWM 分频

保留原有 8 MHz HSE → PLL ×6 → SYSCLK 48 MHz，以及 USB 48 MHz。
APB1 = 24 MHz（分频为 2，TIM2 时钟为 48 MHz）；APB2 = 48 MHz，TIM1 时钟也为 48 MHz。

两个定时器统一：

```text
PSC = 15，ARR = 59999
计数频率 = 48 MHz / (15 + 1) = 3 MHz
PWM 频率 = 3 MHz / (59999 + 1) = 50 Hz
一个 CCR 计数 = 1/3 μs
默认 500～2500 μs → CCR 1500～7500；135° → CCR 4500
```

16 位计数器能容纳 60000 个计数。按默认 270° / 2000 μs 的映射，PWM 量化步进约 0.045°；
这是输出分辨率，实际定位精度还取决于舵机本身、反馈标定和机械回差。
角度使用 `float`，最后一步才对 CCR 四舍五入，保留亚微秒分辨率。
500～2500 μs / 50 Hz 是待实机确认的初始设置，不是特定型号的已验证参数。

ADC 时钟 12 MHz，单次软件触发，采样时间 71.5 cycles。
IOC 保持单通道规则转换；`ADC_Read()` 在每次读取前切换通道，七路每 20 ms 依次读取，无 DMA。
`ADC_Init()` 在首次采样前执行 ADC 校准。

## 文件入口

| 文件 | 用途 |
|---|---|
| `Application/Inc/servo_config.h` | 角度、PWM、ADC 端点、死区和外环参数宏 |
| `Application/Inc/servo.h`、`Application/Src/servo.c` | 单舵机结构体、换算、反馈滤波、位置修正 |
| `Application/Inc/arm.h`、`Application/Src/arm.c` | 七路实例表、六轴批量操作 |
| `Application/Src/adc_io.c`、`pwm_io.c` | ADC 校准/采样、PWM 写入/启停 |
| `Application/Src/uart_io.c`、`usb_io.c` | UART/USB 收发 |
| `Application/Inc/arm_protocol.h` | 未来上位机协议定义入口，尚未接入解析器 |
| `Core/Src/main.c` | 初始化和 20 ms 裸机调度 |

`arm_servos[]` 可直接加入调试器观察窗口。主要字段：

- `feedback_adc`：本次原始 ADC。
- `feedback_deg`：换算并滤波后的反馈角度。
- `target_deg`：用户目标角度；用 `Servo_SetTarget()` 修改。
- `command_deg`：经过外环修正后的实际 PWM 指令角度。
- `error_deg`、`pwm_ticks`：误差和实际 CCR 计数。
- `enabled`、`closed_loop`：输出使能和外环使能。

## 最小用法

上电七路均不启动 PWM，但持续采集反馈。初始化在 `main.c` 的 USER CODE 2 中：

```c
ADC_Init();
UART_Init();
Arm_Init();
```

标定后在上述初始化后添加，例如先调 H3：

```c
Servo_t *joint = &arm_servos[ARM_SERVO_H3];
Servo_SetClosedLoop(joint, 0U);       // 先直接输出 PWM，核对角度和反馈
Servo_SetTarget(joint, 135.0f);
Servo_Enable(joint, 1U);
// 标定结束后再启用外环：Servo_SetClosedLoop(joint, 1U);
```

六轴批量操作：

```c
const float target[ARM_AXIS_COUNT] = {135, 135, 135, 135, 135, 135};
Arm_SetTargets(target);
Arm_Enable(1U);                      // 仅启用前六路，H10 不受影响
// Arm_Enable(0U);                   // 关闭六轴 PWM
// Servo_Enable(&arm_servos[ARM_SERVO_H10_SPARE], 1U); // 单独启用备用路
```

`Arm_Update()` 已由主循环每 20 ms 调用，反馈和控制均在主循环执行。
修改目标无需自行转换 PWM，也无需在通信中断里调用舵机函数。

## 标定与简单闭环

在 `servo_config.h` 修改：

1. `SERVO_PWM_MIN_US/MAX_US`：按实际舵机规格和实测修正。
2. 各路 `SERVO_Hx_ADC_AT_MIN/MAX`：角度两端对应的实测 ADC 值。两端不能相等；反向反馈可填递减端点。
3. `SERVO_ANGLE_MIN_DEG/MAX_DEG`：有效角度边界。
4. `SERVO_DEADBAND_DEG`：误差死区，初始 1°。
5. `SERVO_FEEDBACK_FILTER_ALPHA`：一阶滤波系数，初始 0.25。
6. `SERVO_CORRECTION_GAIN`、`SERVO_CORRECTION_LIMIT_DEG`：每周期修正增益 0.1、最大偏移 ±15°。

换算：`angle = min_angle + (adc - adc_at_min) / (adc_at_max - adc_at_min) * angle_span`。
外环在死区外执行 `command += gain * (target - feedback)`，并限制在目标 ±修正上限及有效角度范围内；
死区内保持原 PWM。它是围绕舵机内置位置环的慢速增量修正，不是完整 PID。
重复设置相同目标不会清空修正量；失效的 ADC 采样会跳过该路本周期控制。
`SERVO_CLOSED_LOOP_DEFAULT` 决定实例初始化时的外环开关；PWM 仍需显式使能。

ADC 端点初始 0/4095 只是占位，必须结合实际反馈标定。
F103 ADC 引脚输入应在 0～VDDA（板上通常为 3.3 V）范围；若舵机反馈超过此范围，需要硬件分压。
软件宏只负责换算，不能代替分压。舵机供电地和 MCU 地应共地。

## 串口与 USB 接口

```c
uint8_t bytes[64];
uint16_t count = USB_Read(bytes, sizeof(bytes));
// 在这里把 count 个字节交给未来的 USB 协议解析器
count = UART_Read(bytes, sizeof(bytes));
// 串口是独立字节流，交给串口解析器

uint8_t usb_result = USB_Send(bytes, count);    // USBD_OK / BUSY / FAIL
HAL_StatusTypeDef uart_result = UART_Send(bytes, count); // HAL_OK / BUSY / ERROR
```

每个端口独立接收缓冲区 256 字节（可用 255），满时丢弃新数据；主循环应及时读取。
串口采用单字节 RX 中断；USB 在 CDC 接收回调中先复制数据再重新挂接端点。
发送一次支持 1～256 字节，接口内部保存数据副本，因此调用者的局部数组可立即复用。
发送非阻塞，忙时由调用者下次重试，不内置发送队列。收发接口供主循环调用。
USB 未枚举时返回 BUSY；CDC 的 GET/SET_LINE_CODING 已补齐。

`arm_protocol.h` 仅预留命令号和目标角度载荷定义；帧头、长度、字节序、校验和解析器仍待上位机设计。
一次 USB 包或一次 `Read()` 不能作为完整协议帧的边界。

## 构建与验证

新增业务代码集中在 `Application/Inc` 和 `Application/Src`；`Core` 与 `USB_DEVICE` 保留 CubeMX 生成代码。
CMake、CMake 模板和 CubeIDE 的 Debug/Release 配置已加入 Application 的头文件及源文件路径。
原 CMakeLists 要求 CMake >= 4.0。本机可用：

```powershell
& 'D:\CMake\bin\cmake.exe' -S . -B build-arm -G Ninja -DCMAKE_BUILD_TYPE=Debug
& 'D:\CMake\bin\cmake.exe' --build build-arm -j 8
```

验证包括 ARM GCC 编译链接，以及主机侧的换算端点、反向 ADC、死区/限幅、重复目标、
简化被控对象收敛、备用通道隔离、接收缓冲回卷/溢出、USB/UART 发送数据寿命和忙状态测试。
这些软件检查不能确认实机脉宽、USB 枚举或舵机实际精度，仍需上板测量。

外设初始化由修改后的 IOC 经 CubeMX 生成，业务接入置于 USER CODE 区。
本机 CubeMX 6.18.1 的 CubeIDE 项目元数据生成曾报路径异常；验证副本改用 Makefile 后返回 OK，
但其 `syscalls.c/sysmem.c` 路径复制仍有异常。交付只采用生成正确的 ADC/TIM/UART/IRQ 外设代码，
并使用原 CMake 完成整机软件编译。未认定本机 CubeMX 的完整工程生成环境已修复。
原工程保留 STM32CubeIDE 工具链选项，CMake 与 IDE 配置仅增加 Application 路径。

硬件参考：[ST RM0008](https://www.st.com/resource/en/reference_manual/rm0008-stm32f103xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)，
本工程随附 STM32F1 HAL 源码。
