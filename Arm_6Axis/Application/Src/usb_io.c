#include "usb_io.h"
#include "ring_buffer.h"
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
static RingBuffer_t rx_ring;
static uint8_t tx_buffer[COMM_BUFFER_SIZE];

uint16_t USB_Read(uint8_t *data, uint16_t capacity)
{
    return RingBuffer_Read(&rx_ring, data, capacity);
}

void USB_OnReceive(const uint8_t *data, uint16_t length)
{
    RingBuffer_Write(&rx_ring, data, length);
}

uint8_t USB_Send(const uint8_t *data, uint16_t length)
{
    uint8_t result = USBD_BUSY;
    if (length == 0U || length > sizeof(tx_buffer)) return USBD_FAIL;

    /* 短暂关闭中断，避免复制数据时 USB 断开导致类数据指针失效 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc != NULL && cdc->TxState == 0U) {
        memcpy(tx_buffer, data, length);
        result = CDC_Transmit_FS(tx_buffer, length);
    }
    __set_PRIMASK(primask);
    return result;
}
