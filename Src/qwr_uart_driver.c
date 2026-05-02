#include "qwr_uart_driver.h"
#include <stdio.h>

UART_HandleTypeDef huart1;

/* HAL callback: configure USART1 GPIO + clock when HAL_UART_Init() runs. */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /* PB6 = USART1_TX, PB7 = USART1_RX, AF7 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void UART1_Init(void)
{
    huart1.Instance              = USART1;
    huart1.Init.BaudRate         = 115200;
    huart1.Init.WordLength       = UART_WORDLENGTH_8B;
    huart1.Init.StopBits         = UART_STOPBITS_1;
    huart1.Init.Parity           = UART_PARITY_NONE;
    huart1.Init.Mode             = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl        = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling     = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling   = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler   = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart1);

    /* Disable stdout line buffering so printf() flushes immediately. */
    setvbuf(stdout, NULL, _IONBF, 0);
}

/* Hook used by syscalls.c::_write() — printf ultimately ends up here. */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* Optional: support scanf() / getchar() over the same UART. */
int __io_getchar(void)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart1, &ch, 1, HAL_MAX_DELAY);
    return ch;
}
