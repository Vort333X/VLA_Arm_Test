# 六轴舵机机械臂底层

STM32F103C8T6 裸机控制，六轴加一路备用，通过 USART1 二进制协议收发关节指令和反馈。

## 接口与轴序

每个舵机插座：1 GND、2 舵机电源、3 PWM、4 ADC 反馈。

| 机械轴 | SWD 实例 / 固件编号 | 插座 | PWM | MCU 引脚 | ADC1 通道 / 引脚 |
|---|---|---|---|---|---|
| J1 | `servo_h3` / 0 | H3 | TIM1_CH1 | PA8 | IN0 / PA0 |
| J2 | `servo_h7` / 1 | H7 | TIM2_CH4 | PB11 | IN6 / PA6 |
| J3 | `servo_h5` / 2 | H5 | TIM1_CH2 | PA9 | IN1 / PA1 |
| J4 | `servo_h10` / 3 | H10 | TIM2_CH1 | PA15 | IN3 / PA3 |
| J5 | `servo_h8` / 4 | H8 | TIM2_CH3 | PB10 | IN5 / PA5 |
| J6 | `servo_h6` / 5 | H6 | TIM2_CH2 | PB3 | IN4 / PA4 |
| 备用 | `servo_h4` / 6 | H4 | TIM1_CH3 | PA10 | IN2 / PA2 |

H9：PB6 = USART1_TX，PB7 = USART1_RX，115200 / 8N1。USB：PA11 DM、PA12 DP。
TIM2 全重映射、USART1 重映射；禁用 JTAG 释放 PA15/PB3，保留 SWD。
轴序按实机从底座到末端排列，批量控制和串口协议共用这一顺序。

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
| `Application/Inc/arm_protocol.h`、`Application/Src/arm_protocol.c` | USART1 二进制帧定义、解析和应答 |
| `Core/Src/main.c` | 初始化和 20 ms 裸机调度 |

按 H 口将独立结构体加入 SWD 观察窗口，例如 `servo_h3`。主要字段如下。

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
Servo_t *joint = &servo_h3;
Servo_SetClosedLoop(joint, 0U);       // 先直接输出 PWM，核对角度和反馈
Servo_SetTarget(joint, 135.0f);
Servo_Enable(joint, 1U);
// 标定结束后再启用外环：Servo_SetClosedLoop(joint, 1U);
```

六轴批量操作：

```c
const float target[ARM_AXIS_COUNT] = {135, 135, 135, 135, 135, 135};
Arm_SetTargets(target);
Arm_Enable(1U);                      // 启用六个工作轴
// Arm_Enable(0U);                   // 关闭六轴 PWM
// Servo_Enable(&servo_h4, 1U);       // 单独启用备用路
```

`Arm_Update()` 已由主循环每 20 ms 调用，反馈和控制均在主循环执行。
修改目标无需自行转换 PWM，也无需在通信中断里调用舵机函数。

## 标定与简单闭环

在 `servo_config.h` 修改：

1. `SERVO_PWM_MIN_US/MAX_US`：按实际舵机规格和实测修正。
2. 各路 `SERVO_Hx_ADC_CALIBRATION`：依次填写 `SERVO_ADC_CALIBRATION(角度A, ADC_A, 角度B, ADC_B)`。角度增大时 ADC 减小，照实填写，换算会自动采用负斜率。
3. `SERVO_ANGLE_MIN_DEG/MAX_DEG`：有效角度边界。
4. `SERVO_DEADBAND_DEG`：误差死区，初始 1°。
5. `SERVO_FEEDBACK_FILTER_ALPHA`：一阶滤波系数，初始 0.25。
6. `SERVO_CORRECTION_GAIN`、`SERVO_CORRECTION_LIMIT_DEG`：每周期修正增益 0.1、最大偏移 ±15°。

每次只调一轴，在装配后的安全行程内选两个相隔较远的角度。等舵机停稳后，记下两点的角度和 ADC 原始值，两点的 ADC 必须不同。初调可用网页目标角来对齐命令和反馈。需要实际机械角时，用量角器实测关节相对中位的有符号偏角，叠加 `135°` 后填写。标完再用两点之间的第三个位置复核。

换算：`angle = angle_a + (adc - adc_a) / (adc_b - adc_a) * (angle_b - angle_a)`。
外环在死区外执行 `command += gain * (target - feedback)`，并限制在目标 ±修正上限及有效角度范围内；
死区内保持原 PWM。它是围绕舵机内置位置环的慢速增量修正，不是完整 PID。
重复设置相同目标不会清空修正量；失效的 ADC 采样会跳过该路本周期控制。
`SERVO_CLOSED_LOOP_DEFAULT` 决定实例初始化时的外环开关；PWM 仍需显式使能。

ADC 初值 0/4095 用于占位，按实测标定后再试外环。
F103 ADC 引脚输入应在 0～VDDA（板上通常为 3.3 V）范围；若舵机反馈超过此范围，需要硬件分压。
软件宏只负责换算，不能代替分压。舵机供电地和 MCU 地应共地。

## USART1 协议 v2

USART1 使用 115200 / 8N1，无流控，按字节流收发。解析器按帧头和载荷长度组帧，单次读取可能包含半帧或多帧。板载 USB CDC 保留为独立接口。

| 偏移 | 字节数 | 字段 |
|---|---:|---|
| 0 | 2 | 同步字 `AA 55` |
| 2 | 1 | 版本 `02` |
| 3 | 1 | 消息类型 |
| 4 | 2 | `seq` 请求序号，应答原样返回 |
| 6 | 2 | `len` 载荷字节数，允许 `0～512` |
| 8 | `len` | 载荷 |
| `8 + len` | 2 | CRC-16/CCITT-FALSE，从版本字节起算到载荷末尾 |

所有多字节整数按小端序编码，最大整帧 522 字节。CRC 多项式 `0x1021`，初值 `0xFFFF`，输入输出不反射，结果异或 `0x0000`。校验字符串 `123456789` 的结果是 `0x29B1`，在线上写作 `B1 29`。

从底座到末端的轴序为 `H3、H7、H5、H10、H8、H6`，轴掩码 `mask` 的低 6 位按此顺序对应六轴，H4 为备用路。角度字段为无符号 16 位整数，每单位 `0.01°`，例如 `140°` 编码为十进制 `14000`。

V2 沿用 V1 的帧结构，为新轴序分配独立的协议版本。网页和 VLA 客户端需使用 V2 轴序与版本字节。

| 请求类型 | 载荷 | 应答 |
|---|---|---|
| `0x01 SET_TARGETS` | `mask:u8 + 6 × angle:u16`，共 13 字节，只更新选中轴，未选轴字段可填零 | `0x80 ACK` |
| `0x02 SET_ENABLE` | `mask:u8 + enable:u8`，`0` 关闭 PWM，`1` 使能 | `0x80 ACK` |
| `0x03 READ_FEEDBACK` | 空载荷 | `0x81 FEEDBACK` |
| `0x04 PING` | 任意 `0～512` 字节 | `0x84 PONG`，原样回传载荷 |

`ACK` 的两个字节依次为请求类型和状态码。状态 `0` 表示指令已处理，`1` 表示载荷长度错误，`2` 表示角度、使能值或轴掩码无效，`3` 表示未知命令。ACK 只确认指令已处理，末端到位需要结合反馈判断。

`FEEDBACK` 共 38 字节，前两个字节为使能掩码、已采样掩码，随后按轴序重复 `目标角度:u16 + 反馈角度:u16 + ADC 原始值:u16`。已采样位表示曾获得有效 ADC 样本。反馈角度精度取决于各轴实测标定。

上电后 PWM 默认关闭。上位机可以先发送目标，再显式使能选中的轴。当前帧直接设置舵机目标，装配后的各轴有效行程和过渡轨迹由测试上位机选定。串口断开后，固件保持最后的目标与 PWM 使能状态。

主循环每轮最多解析 128 字节，串口发送忙时下一轮继续尝试。收发均通过中断，半帧超过 100 ms 无后续数据会丢弃，校验失败或不支持的协议版本静默丢弃。USART1 的 RX 缓冲为 1024 字节，可用 1023 字节，TX 缓冲可一次容纳最大 522 字节整帧。

## 网页与 VLA 上位机接入

网页使用桌面 Chrome 的 Web Serial，点击连接后选择接到 H9 的 USB 转串口模块对应的 COM 口。网页用 `Uint8Array` 和 `DataView` 生成上述二进制帧，读取时将字节持续送入组帧器，再按 `seq` 匹配应答。第一版使用一问一答，同时只保留一个待应答请求，建议收到应答后再发送下一帧，等待超时可从 300 ms 起调。

页面首次连接先发送 `PING` 校验通信，再用 `READ_FEEDBACK` 更新关节状态。单轴调试时通过轴掩码选择关节。末端打点或 VLA 输出在上位机进行逆解和轨迹处理，逐帧发送同一格式的关节目标。VLA 进程可通过普通 COM 口客户端接入，与网页使用同一份协议。一个 COM 口同一时间由一个客户端打开。

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
