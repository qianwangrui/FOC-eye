#ifndef _QWR_UART_DRIVER_H_
#define _QWR_UART_DRIVER_H_

#include "stm32g4xx_hal.h"

/* USART1 on PB6 (TX) / PB7 (RX), 115200 8N1.
 * Used to redirect printf() to a serial terminal. */
void UART1_Init(void);

#endif /* _QWR_UART_DRIVER_H_ */
