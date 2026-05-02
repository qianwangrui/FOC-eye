#include "qwr_INA240_driver.h"

#define INA240A1_GAIN       20.0f
#define VDDA                3.3f
#define ADC_RESOLUTION      4096.0f
#define VREF_HALF           (VDDA / 2.0f)
#define SHUNT_RESISTOR      0.05f

ADC_HandleTypeDef hadc1;

void INA240_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA2 - IU 电流采样 (ADC1_IN3) */
    GPIO_InitTypeDef gpio_iu = {0};
    gpio_iu.Pin  = GPIO_PIN_2;
    gpio_iu.Mode = GPIO_MODE_ANALOG;
    gpio_iu.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio_iu);

    /* PB1 - IV 电流采样 (ADC1_IN12) */
    GPIO_InitTypeDef gpio_iv = {0};
    gpio_iv.Pin  = GPIO_PIN_1;
    gpio_iv.Mode = GPIO_MODE_ANALOG;
    gpio_iv.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio_iv);
}

void INA240_ADC_Init(void)
{
    __HAL_RCC_ADC12_CLK_ENABLE();

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode      = DISABLE;
    HAL_ADC_Init(&hadc1);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
}

static uint16_t INA240_ReadADC(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return val;
}

float INA240_ReadCurrent_IU(void)
{
    uint16_t adc_val = INA240_ReadADC(ADC_CHANNEL_3);
    float voltage = (float)adc_val / ADC_RESOLUTION * VDDA;
    float current = (voltage - VREF_HALF) / (INA240A1_GAIN * SHUNT_RESISTOR);
    return current;
}

float INA240_ReadCurrent_IV(void)
{
    uint16_t adc_val = INA240_ReadADC(ADC_CHANNEL_12);
    float voltage = (float)adc_val / ADC_RESOLUTION * VDDA;
    float current = (voltage - VREF_HALF) / (INA240A1_GAIN * SHUNT_RESISTOR);
    return current;
}
