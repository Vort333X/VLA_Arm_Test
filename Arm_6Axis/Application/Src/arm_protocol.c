#include "arm_protocol.h"
#include "arm.h"
#include "usb_io.h"
#include <string.h>

#define RX_BUDGET_BYTES      128U
#define RX_IDLE_TIMEOUT_MS  100U
#define ARM_AXIS_MASK        ((1U << ARM_AXIS_COUNT) - 1U)

static uint8_t rx_frame[ARM_PROTOCOL_FRAME_SIZE];
static uint8_t tx_frame[ARM_PROTOCOL_FRAME_SIZE];
static uint16_t rx_size;
static uint16_t rx_expected;
static uint16_t tx_size;
static uint32_t last_rx_ms;
static uint32_t usb_session;

static uint16_t ReadU16(const uint8_t *bytes)
{
    return (uint16_t)(bytes[0] | ((uint16_t)bytes[1] << 8));
}

static void WriteU16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
}

/* CRC-16/CCITT-FALSE，覆盖版本到载荷末尾 */
static uint16_t Crc16(const uint8_t *bytes, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    while (length-- != 0U) {
        crc ^= (uint16_t)*bytes++ << 8;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
        }
    }
    return crc;
}

/* 载荷先写入发送缓冲区，此处补齐帧头和校验 */
static void QueueReply(uint8_t type, uint16_t seq, uint16_t length)
{
    tx_frame[0] = ARM_PROTOCOL_SYNC_1;
    tx_frame[1] = ARM_PROTOCOL_SYNC_2;
    tx_frame[2] = ARM_PROTOCOL_VERSION;
    tx_frame[3] = type;
    WriteU16(&tx_frame[4], seq);
    WriteU16(&tx_frame[6], length);
    uint16_t end = ARM_PROTOCOL_HEADER_SIZE + length;
    WriteU16(&tx_frame[end], Crc16(&tx_frame[2], end - 2U));
    tx_size = end + 2U;
}

static ArmProtocolStatus_t SetTargets(const uint8_t *payload, uint16_t length)
{
    if (length != 1U + 2U * ARM_AXIS_COUNT) return ARM_ACK_BAD_LENGTH;
    uint8_t mask = payload[0];
    if (mask == 0U || (mask & ~ARM_AXIS_MASK)) return ARM_ACK_BAD_VALUE;

    float degrees[ARM_AXIS_COUNT];
    /* 先检查所有选中的轴，整帧通过后再更新目标 */
    for (uint8_t i = 0U; i < ARM_AXIS_COUNT; ++i) {
        degrees[i] = (float)ReadU16(&payload[1U + 2U * i]) / ARM_PROTOCOL_ANGLE_SCALE;
        if ((mask & (1U << i)) &&
            (degrees[i] < SERVO_ANGLE_MIN_DEG || degrees[i] > SERVO_ANGLE_MAX_DEG)) {
            return ARM_ACK_BAD_VALUE;
        }
    }
    for (uint8_t i = 0U; i < ARM_AXIS_COUNT; ++i) {
        if (mask & (1U << i)) Servo_SetTarget(&arm_servos[i], degrees[i]);
    }
    return ARM_ACK_OK;
}

static ArmProtocolStatus_t SetEnable(const uint8_t *payload, uint16_t length)
{
    if (length != 2U) return ARM_ACK_BAD_LENGTH;
    uint8_t mask = payload[0];
    uint8_t enable = payload[1];
    if (mask == 0U || (mask & ~ARM_AXIS_MASK) || enable > 1U) return ARM_ACK_BAD_VALUE;

    for (uint8_t i = 0U; i < ARM_AXIS_COUNT; ++i) {
        Servo_t *servo = &arm_servos[i];
        if ((mask & (1U << i)) && servo->enabled != enable) Servo_Enable(servo, enable);
    }
    return ARM_ACK_OK;
}

static void QueueFeedback(uint16_t seq)
{
    uint8_t *payload = &tx_frame[ARM_PROTOCOL_HEADER_SIZE];
    payload[0] = 0U;
    payload[1] = 0U;
    for (uint8_t i = 0U; i < ARM_AXIS_COUNT; ++i) {
        const Servo_t *servo = &arm_servos[i];
        if (servo->enabled) payload[0] |= (uint8_t)(1U << i);
        if (servo->feedback_ready) payload[1] |= (uint8_t)(1U << i);

        uint8_t *axis = &payload[2U + 6U * i];
        WriteU16(&axis[0], (uint16_t)(servo->target_deg * ARM_PROTOCOL_ANGLE_SCALE + 0.5f));
        WriteU16(&axis[2], (uint16_t)(servo->feedback_deg * ARM_PROTOCOL_ANGLE_SCALE + 0.5f));
        WriteU16(&axis[4], servo->feedback_adc);
    }
    QueueReply(ARM_MSG_FEEDBACK, seq, 2U + 6U * ARM_AXIS_COUNT);
}

static void HandleFrame(void)
{
    uint16_t end = rx_size - 2U;
    if (ReadU16(&rx_frame[end]) != Crc16(&rx_frame[2], end - 2U)) return;

    uint8_t type = rx_frame[3];
    uint16_t seq = ReadU16(&rx_frame[4]);
    uint16_t length = ReadU16(&rx_frame[6]);
    const uint8_t *payload = &rx_frame[ARM_PROTOCOL_HEADER_SIZE];
    uint8_t *reply = &tx_frame[ARM_PROTOCOL_HEADER_SIZE];
    ArmProtocolStatus_t status;

    switch (type) {
        case ARM_CMD_SET_TARGETS:
            status = SetTargets(payload, length);
            break;
        case ARM_CMD_SET_ENABLE:
            status = SetEnable(payload, length);
            break;
        case ARM_CMD_READ_FEEDBACK:
            if (length == 0U) {
                QueueFeedback(seq);
                return;
            }
            status = ARM_ACK_BAD_LENGTH;
            break;
        case ARM_CMD_PING:
            memcpy(reply, payload, length);
            QueueReply(ARM_MSG_PONG, seq, length);
            return;
        default:
            status = ARM_ACK_UNKNOWN_COMMAND;
            break;
    }

    reply[0] = type;
    reply[1] = (uint8_t)status;
    QueueReply(ARM_MSG_ACK, seq, 2U);
}

static void ParseByte(uint8_t byte)
{
    /* 找到同步字后按长度收帧，载荷中允许出现同步字 */
    if (rx_size == 0U) {
        if (byte == ARM_PROTOCOL_SYNC_1) rx_frame[rx_size++] = byte;
        return;
    }
    if (rx_size == 1U && byte != ARM_PROTOCOL_SYNC_2) {
        rx_size = (byte == ARM_PROTOCOL_SYNC_1) ? 1U : 0U;
        return;
    }
    if (rx_size == 2U && byte != ARM_PROTOCOL_VERSION) {
        rx_size = (byte == ARM_PROTOCOL_SYNC_1) ? 1U : 0U;
        return;
    }

    rx_frame[rx_size++] = byte;
    if (rx_size < ARM_PROTOCOL_HEADER_SIZE) return;
    if (rx_size == ARM_PROTOCOL_HEADER_SIZE) {
        uint16_t length = ReadU16(&rx_frame[6]);
        if (length > ARM_PROTOCOL_MAX_PAYLOAD) {
            rx_size = (byte == ARM_PROTOCOL_SYNC_1) ? 1U : 0U;
            return;
        }
        rx_expected = ARM_PROTOCOL_HEADER_SIZE + length + 2U;
    }
    if (rx_size == rx_expected) {
        HandleFrame();
        rx_size = 0U;
    }
}

void Arm_ProtocolPoll(uint32_t now_ms)
{
    uint32_t session = USB_Session();
    if (session != usb_session) {
        usb_session = session;
        rx_size = 0U;
        tx_size = 0U;
    }
    if (tx_size != 0U) {
        if (USB_Send(tx_frame, tx_size) == USBD_BUSY) return;
        tx_size = 0U;
    }
    if ((uint32_t)(now_ms - last_rx_ms) >= RX_IDLE_TIMEOUT_MS) rx_size = 0U;

    for (uint16_t i = 0U; i < RX_BUDGET_BYTES && tx_size == 0U; ++i) {
        uint8_t byte;
        if (USB_Read(&byte, 1U) == 0U || USB_Session() != session) break;
        ParseByte(byte);
        last_rx_ms = now_ms;
    }
}
