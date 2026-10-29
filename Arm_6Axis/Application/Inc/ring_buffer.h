#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>

#define COMM_BUFFER_SIZE 256U

/* 由单个中断写入、主循环读取，缓冲区满时丢弃新数据 */
typedef struct {
    volatile uint8_t data[COMM_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer_t;

static inline void RingBuffer_Write(RingBuffer_t *ring, const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; ++i) {
        uint16_t next = (ring->head + 1U) % COMM_BUFFER_SIZE;
        if (next == ring->tail) break;
        ring->data[ring->head] = data[i];
        ring->head = next;
    }
}

static inline uint16_t RingBuffer_Read(RingBuffer_t *ring, uint8_t *data, uint16_t capacity)
{
    uint16_t count = 0U;
    while (count < capacity && ring->tail != ring->head) {
        data[count++] = ring->data[ring->tail];
        ring->tail = (ring->tail + 1U) % COMM_BUFFER_SIZE;
    }
    return count;
}

#endif
