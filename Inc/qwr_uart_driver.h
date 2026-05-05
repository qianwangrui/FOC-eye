#ifndef _QWR_UART_DRIVER_H_
#define _QWR_UART_DRIVER_H_

#include "stm32g4xx_hal.h"

/* USART1 on PB6 (TX) / PB7 (RX), 115200 8N1.
 * Used to redirect printf() to a serial terminal. */
void UART1_Init(void);

/* Start interrupt-driven RX. Each received line (terminated by '\n' or '\r')
 * is parsed as a floating-point number and ADDED to g_foc.pos_ref_deg.
 * E.g. sending "10\n" advances the target angle by 10 degrees;
 *      sending "-90\n" rewinds by 90 degrees.
 * Call AFTER UART1_Init() and AFTER FOC is up. */
void UART1_StartCmdRx(void);
int  UART1_GetByte(void);

#endif /* _QWR_UART_DRIVER_H_ */
