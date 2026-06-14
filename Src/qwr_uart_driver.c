#include "qwr_uart_driver.h"
#include "debug_cpu.h"
#include <stdio.h>

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart1;

/* ---- Bare-metal RX ring buffer (filled by USART3 RXNE ISR) -------- */
#define UART_RX_BUF_SIZE  64          /* must be power of 2 */
static volatile uint8_t s_rx_buf[UART_RX_BUF_SIZE];
static volatile uint8_t s_rx_head;    /* written by ISR  */
static volatile uint8_t s_rx_tail;    /* read by main    */
volatile uint32_t g_uart_rx_cnt;      /* debug: total bytes received */
volatile uint32_t g_uart_isr_cnt;     /* debug: total ISR entries */
volatile uint32_t g_uart_cr1_dbg;     /* debug: CR1 snapshot after init */

/* HAL callback: configure GPIO + clock for USART1 / USART3 when
 * HAL_UART_Init() runs. */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};

    if (huart->Instance == USART3) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_USART3_CLK_ENABLE();

        /* PB10 = USART3_TX, PB11 = USART3_RX, AF7 */
        gpio.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_PULLUP;
        gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &gpio);
        return;
    }

    if (huart->Instance == USART1) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();

        /* PB6 = USART1_TX, PB7 = USART1_RX, AF7 */
        gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_PULLUP;
        gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOB, &gpio);
        return;
    }
}

void UART3_Init(void)
{
    huart3.Instance              = USART3;
    huart3.Init.BaudRate         = 115200;
    huart3.Init.WordLength       = UART_WORDLENGTH_8B;
    huart3.Init.StopBits         = UART_STOPBITS_1;
    huart3.Init.Parity           = UART_PARITY_NONE;
    huart3.Init.Mode             = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl        = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling     = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling   = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler   = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart3);
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

/* Hook used by syscalls.c::_write() — printf ultimately ends up here.
 * VOFA+ telemetry is sent on USART1 (PB6 TX). */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* Optional: support scanf() / getchar() over the command UART (USART3). */
int __io_getchar(void)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart3, &ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* -------------------------------------------------------------------------
 * Bare-metal RXNE interrupt: just stuffs bytes into the ring buffer.
 * No HAL state machine, no callbacks, no lock — cannot get stuck.
 * ------------------------------------------------------------------------- */
void UART3_StartCmdRx(void)
{
    s_rx_head = 0;
    s_rx_tail = 0;

    /* Clear any pending RX / error flags before enabling interrupts. */
    USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF
               | USART_ICR_PECF  | USART_ICR_IDLECF;
    (void)USART3->RDR;   /* dummy read to clear RXNE if set */

    /* Enable only RXNE interrupt (no EIE — we handle ORE inside ISR). */
    USART3->CR1 |= USART_CR1_RXNEIE_RXFNEIE;

    HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);

    g_uart_cr1_dbg = USART3->CR1;   /* snapshot for debugging */
}

void USART3_IRQHandler(void)
{
    debug_cpu_busy();
    g_uart_isr_cnt++;
    uint32_t isr = USART3->ISR;

    /* Clear any error flags (overrun / framing / noise). */
    if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE))
        USART3->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;

    /* Read every available byte into the ring buffer. */
    if (isr & USART_ISR_RXNE_RXFNE) {
        uint8_t byte = (uint8_t)(USART3->RDR & 0xFF);
        g_uart_rx_cnt++;
        uint8_t next = (s_rx_head + 1) & (UART_RX_BUF_SIZE - 1);
        if (next != s_rx_tail) {          /* buffer not full */
            s_rx_buf[s_rx_head] = byte;
            s_rx_head = next;
        }
    }
    debug_cpu_idle();
}

/* Called from main loop: returns -1 if empty, else the byte. */
int UART3_GetByte(void)
{
    if (s_rx_tail == s_rx_head) return -1;
    uint8_t byte = s_rx_buf[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1) & (UART_RX_BUF_SIZE - 1);
    return byte;
}
