#include "uart_io.h"
#include "ring_buffer.h"
#include <string.h>

static RingBuffer_t rx_ring;
static uint8_t rx_byte;
static uint8_t tx_buffer[COMM_BUFFER_SIZE];

void UART_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

uint16_t UART_Read(uint8_t *data, uint16_t capacity)
{
    return RingBuffer_Read(&rx_ring, data, capacity);
}

HAL_StatusTypeDef UART_Send(const uint8_t *data, uint16_t length)
{
    if (length == 0U || length > sizeof(tx_buffer)) return HAL_ERROR;
    if (huart1.gState != HAL_UART_STATE_READY) return HAL_BUSY;
    memcpy(tx_buffer, data, length);
    return HAL_UART_Transmit_IT(&huart1, tx_buffer, length);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance != USART1) return;
    RingBuffer_Write(&rx_ring, &rx_byte, 1U);
    HAL_UART_Receive_IT(uart, &rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart->Instance != USART1) return;
    HAL_UART_AbortReceive(uart);
    HAL_UART_Receive_IT(uart, &rx_byte, 1U);
}
