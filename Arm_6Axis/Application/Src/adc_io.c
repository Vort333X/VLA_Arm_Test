#include "adc_io.h"
#include "servo_config.h"

void ADC_Init(void)
{
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
        Error_Handler();
    }
}

HAL_StatusTypeDef ADC_Read(uint32_t channel, uint16_t *value)
{
    ADC_ChannelConfTypeDef config = {0};
    HAL_StatusTypeDef status;

    /* 每次读取前选择输入通道，再由软件触发一次转换 */
    config.Channel = channel;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLE_TIME;
    status = HAL_ADC_ConfigChannel(&hadc1, &config);
    if (status != HAL_OK) return status;
    status = HAL_ADC_Start(&hadc1);
    if (status != HAL_OK) return status;
    status = HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS);
    if (status == HAL_OK) *value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return status;
}
