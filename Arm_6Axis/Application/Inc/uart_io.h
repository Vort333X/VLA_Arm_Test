#ifndef UART_IO_H
#define UART_IO_H

#include "usart.h"

void UART_Init(void);
uint16_t UART_Read(uint8_t *data, uint16_t capacity);
/* 在主循环调用，非阻塞发送，内部复制数据，上次发送未完成时返回 HAL_BUSY */
HAL_StatusTypeDef UART_Send(const uint8_t *data, uint16_t length);

#endif
