#ifndef DEBUG_CPU_H
#define DEBUG_CPU_H

#include "stm32g4xx_hal.h"

/* Set 0 to strip all measurement code. TP1 on schematic = PC13. */
#define DEBUG_CPU_ENABLE  1

void debug_cpu_init(void);

/* Nesting busy/idle markers for scope-based CPU idle measurement.
 * TP1 LOW = idle, TP1 HIGH = busy. Scope duty cycle on LOW ≈ idle rate (%). */
void debug_cpu_busy(void);
void debug_cpu_idle(void);

#endif /* DEBUG_CPU_H */
