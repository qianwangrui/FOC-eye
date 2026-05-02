#include "qwr_MT6701_driver.h"

SPI_HandleTypeDef hspi1;
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
   __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA4 - CSN 片选 (GPIO 手动控制, 默认高电平=不选中) */
    GPIO_InitTypeDef gpio_csn = {0};
    gpio_csn.Pin   = GPIO_PIN_4;
    gpio_csn.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_csn.Pull  = GPIO_PULLUP;
    gpio_csn.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_csn);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    /* PA5 - SPI1_SCK 时钟 (AF5) */
    GPIO_InitTypeDef gpio_sck = {0};
    gpio_sck.Pin       = GPIO_PIN_5;
    gpio_sck.Mode      = GPIO_MODE_AF_PP;
    gpio_sck.Pull      = GPIO_NOPULL;
    gpio_sck.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_sck.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio_sck);

    /* PA6 - SPI1_MISO 数据输入 (AF5) */
    GPIO_InitTypeDef gpio_miso = {0};
    gpio_miso.Pin       = GPIO_PIN_6;
    gpio_miso.Mode      = GPIO_MODE_AF_PP;
    gpio_miso.Pull      = GPIO_NOPULL;
    gpio_miso.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio_miso.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &gpio_miso);
}


void MT6701_SPI_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES_RXONLY;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi1);
}

/* Read the full 24-bit SSI frame and decode the three fields.
 * Pass NULL for any field you don't care about.
 *
 * 24-bit frame layout (MSB first):
 *   bit 23..10 : D[13:0]  14-bit angle
 *   bit  9..6  : Mg[3:0]  4-bit magnetic status
 *   bit  5..0  : CRC[5:0] 6-bit CRC
 */
void MT6701_ReadFrame(uint16_t *angle14, uint8_t *status4, uint8_t *crc6)
{
    uint8_t buf[3] = {0};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Receive(&hspi1, buf, 3, 10);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    uint32_t raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

    if (angle14) *angle14 = (raw >> 10) & 0x3FFF;
    if (status4) *status4 = (raw >>  6) & 0x0F;
    if (crc6)    *crc6    =  raw        & 0x3F;
}

uint16_t MT6701_ReadAngle_SSI(void)
{
    uint16_t angle;
    MT6701_ReadFrame(&angle, NULL, NULL);
    return angle;
}

uint8_t MT6701_GetStatus(void)
{
    uint8_t status;
    MT6701_ReadFrame(NULL, &status, NULL);
    return status;
}

float MT6701_GetAngleDeg(void)
{
    uint16_t raw = MT6701_ReadAngle_SSI();
    return (float)raw / 16384.0f * 360.0f;
}