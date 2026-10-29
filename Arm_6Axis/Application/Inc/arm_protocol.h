#ifndef ARM_PROTOCOL_H
#define ARM_PROTOCOL_H

#include <stdint.h>

/* 预留 USB 和串口上位机协议定义，暂未启用解析器
 * 单个 USB 数据包或一次 USB_Read()/UART_Read() 的结果不一定是完整协议帧
 * 实现协议前，先在此定义帧格式、长度、字节序和校验方式 */
#define ARM_PROTOCOL_VERSION 1U
#define ARM_PROTOCOL_ANGLE_SCALE 100U /* 建议以 0.01 度作为整数角度单位 */

typedef enum {
    ARM_CMD_SET_TARGETS = 0x01,
    ARM_CMD_SET_ENABLE = 0x02,
    ARM_CMD_READ_FEEDBACK = 0x03,
    ARM_MSG_FEEDBACK = 0x81
} ArmProtocolCommand_t;

/* 仅为建议的载荷定义，应逐字段序列化，不直接按结构体大小发送 */
typedef struct {
    uint16_t angle_cdeg[6];
} ArmProtocolTargets_t;

#endif
