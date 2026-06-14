#include "debug_cpu.h"

#if DEBUG_CPU_ENABLE

/* TP1 on board schematic: PC13. */
#define DEBUG_CPU_GPIO_PORT   GPIOC
#define DEBUG_CPU_GPIO_PIN    GPIO_PIN_13

static uint8_t s_busy_nest;

void debug_cpu_init(void)
{
    s_busy_nest = 0U;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = DEBUG_CPU_GPIO_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DEBUG_CPU_GPIO_PORT, &gpio);

    /* Start idle (LOW). */
    DEBUG_CPU_GPIO_PORT->BSRR = (uint32_t)DEBUG_CPU_GPIO_PIN << 16U;
}

void debug_cpu_busy(void)
{
    if (s_busy_nest++ == 0U) {
        DEBUG_CPU_GPIO_PORT->BSRR = DEBUG_CPU_GPIO_PIN;
    }
}

void debug_cpu_idle(void)
{
    if (s_busy_nest > 0U && --s_busy_nest == 0U) {
        DEBUG_CPU_GPIO_PORT->BSRR = (uint32_t)DEBUG_CPU_GPIO_PIN << 16U;
    }
}

#else /* DEBUG_CPU_ENABLE */

void debug_cpu_init(void) {}
void debug_cpu_busy(void) {}
void debug_cpu_idle(void) {}

#endif /* DEBUG_CPU_ENABLE */
