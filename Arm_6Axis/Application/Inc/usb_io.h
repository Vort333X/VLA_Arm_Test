#ifndef USB_IO_H
#define USB_IO_H

#include "usbd_cdc_if.h"

uint16_t USB_Read(uint8_t *data, uint16_t capacity);
/* 在主循环调用，非阻塞发送，内部复制数据，上次发送未完成时返回 USBD_BUSY */
uint8_t USB_Send(const uint8_t *data, uint16_t length);
/* 仅在 CDC_Receive_FS 中调用，先保存接收数据，再重新挂接 OUT 端点 */
void USB_OnReceive(const uint8_t *data, uint16_t length);

#endif
