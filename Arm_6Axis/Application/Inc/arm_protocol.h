#ifndef ARM_PROTOCOL_H
#define ARM_PROTOCOL_H

#include <stdint.h>

/* 帧头 | 版本 | 类型 | 序号 | 载荷长度 | 载荷 | CRC16，多字节字段均为小端序 */
#define ARM_PROTOCOL_SYNC_1  0xAAU
#define ARM_PROTOCOL_SYNC_2  0x55U
#define ARM_PROTOCOL_VERSION 1U
#define ARM_PROTOCOL_MAX_PAYLOAD 512U
#define ARM_PROTOCOL_HEADER_SIZE 8U
#define ARM_PROTOCOL_FRAME_SIZE (ARM_PROTOCOL_HEADER_SIZE + ARM_PROTOCOL_MAX_PAYLOAD + 2U)
#define ARM_PROTOCOL_ANGLE_SCALE 100U /* 每个整数单位对应 0.01 度 */

typedef enum {
    ARM_CMD_SET_TARGETS = 0x01,
    ARM_CMD_SET_ENABLE = 0x02,
    ARM_CMD_READ_FEEDBACK = 0x03,
    ARM_CMD_PING = 0x04,
    ARM_MSG_ACK = 0x80,
    ARM_MSG_FEEDBACK = 0x81,
    ARM_MSG_PONG = 0x84
} ArmProtocolCommand_t;

typedef enum {
    ARM_ACK_OK = 0,
    ARM_ACK_BAD_LENGTH = 1,
    ARM_ACK_BAD_VALUE = 2,
    ARM_ACK_UNKNOWN_COMMAND = 3
} ArmProtocolStatus_t;

/* 在主循环调用，每次最多处理一批接收字节，发送忙时留待下一轮 */
void Arm_ProtocolPoll(uint32_t now_ms);

#endif
