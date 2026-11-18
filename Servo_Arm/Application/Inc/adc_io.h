#ifndef ADC_IO_H
#define ADC_IO_H

#include "adc.h"

void ADC_Init(void);
HAL_StatusTypeDef ADC_Read(uint32_t channel, uint16_t *value);

#endif
