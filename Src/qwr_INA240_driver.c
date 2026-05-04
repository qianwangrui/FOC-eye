#include "qwr_INA240_driver.h"

/* ============================================================================
 * Synchronous current sampling for FOC.
 *
 * Pipeline:
 *   TIM1 update event (PWM 谷底, all low-side MOSFETs conducting)
 *     -> TIM1 TRGO
 *     -> ADC1 hardware-triggered scan of CH3 (IU), CH12 (IV)
 *     -> 8x hardware oversampling per channel (right-shift 3 -> 12-bit avg)
 *     -> DMA1 Channel 1 (circular) writes results to s_adc_buf[2]
 *
 * The CPU is NEVER involved in the conversion: no polling, no interrupt.
 * The FOC ISR (also fired by TIM1 update) just reads s_adc_buf[].
 * Because conversion takes ~10 us and ISR reads at t=0 of the same UEV,
 * the read returns the previous PWM cycle's data (one-sample latency,
 * standard for current-sampled FOC). The two phases are sampled at the
 * same DMA cycle, so iu/iv are coherent (no inter-channel skew beyond a
 * couple of ADC clocks).
 * ========================================================================= */

#define INA240A1_GAIN       20.0f
#define VDDA                3.3f
#define ADC_RESOLUTION      4096.0f
#define VREF_HALF           (VDDA / 2.0f)
#define SHUNT_RESISTOR      0.05f

ADC_HandleTypeDef hadc1;
static DMA_HandleTypeDef hdma_adc1;

/* DMA target buffer, written by hardware on every TIM1 update event.
 * Layout matches the ADC scan rank order:
 *   s_adc_buf[0] = IU (rank 1, channel 3)
 *   s_adc_buf[1] = IV (rank 2, channel 12)
 * Marked volatile because the CPU reads it concurrently with DMA writes. */
static volatile uint16_t s_adc_buf[2];

void INA240_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;

    /* PA2 -> ADC1_IN3 (IU) */
    gpio.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PB1 -> ADC1_IN12 (IV) */
    gpio.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void INA240_ADC_Init(void)
{
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* ---- DMA1 Channel 1: peripheral -> memory, circular, 16-bit ---- */
    hdma_adc1.Instance                 = DMA1_Channel1;
    hdma_adc1.Init.Request             = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    HAL_DMA_Init(&hdma_adc1);

    /* ---- ADC1: scan 2 channels, externally triggered by TIM1 TRGO ---- */
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;   /* 170/4 = 42.5 MHz */
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation      = 0;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 2;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T1_TRGO;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;

    /* Hardware oversampling: each external trigger fires 8 conversions per
     * channel, summed and right-shifted by 3 -> 12-bit averaged result.
     * Total time per scan: 2 channels * 8 samples * (24.5 + 12.5) cycles
     *                    = 592 ADC cycles @ 42.5 MHz = ~14 us, well under
     *                      the 50 us PWM period. */
    hadc1.Init.OversamplingMode               = ENABLE;
    hadc1.Init.Oversampling.Ratio             = ADC_OVERSAMPLING_RATIO_8;
    hadc1.Init.Oversampling.RightBitShift     = ADC_RIGHTBITSHIFT_3;
    hadc1.Init.Oversampling.TriggeredMode     = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
    hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
    HAL_ADC_Init(&hadc1);

    /* Link DMA to ADC handle (HAL needs this for state tracking). */
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    /* ---- Sequence configuration: rank 1 = IU (CH3), rank 2 = IV (CH12) ---- */
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;

    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank    = ADC_REGULAR_RANK_1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank    = ADC_REGULAR_RANK_2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

    /* Arm the ADC: from now on, every TIM1 update event triggers a full
     * 2-channel oversampled scan whose results land in s_adc_buf[]. */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adc_buf, 2);
}

static inline float adc_to_current(uint16_t raw)
{
    float voltage = (float)raw * (VDDA / ADC_RESOLUTION);
    return (voltage - VREF_HALF) / (INA240A1_GAIN * SHUNT_RESISTOR);
}

float INA240_ReadCurrent_IU(void)
{
    return adc_to_current(s_adc_buf[0]);
}

float INA240_ReadCurrent_IV(void)
{
    return adc_to_current(s_adc_buf[1]);
}
