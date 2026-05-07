#ifndef _QWR_UART_DRIVER_H_
#define _QWR_UART_DRIVER_H_

#include "stm32g4xx_hal.h"

/* USART3 on PB10 (TX) / PB11 (RX), 115200 8N1.
 * Used as the command-RX channel from VOFA / terminal. */
void UART3_Init(void);

/* USART1 on PB6 (TX) / PB7 (RX), 115200 8N1.
 * TX-only path used to stream VOFA+ telemetry (printf is redirected here). */
void UART1_Init(void);

/* Start interrupt-driven RX. Each received line (terminated by '\n' or '\r')
 * is parsed as a floating-point number and ADDED to g_foc.pos_ref_deg.
 * E.g. sending "10\n" advances the target angle by 10 degrees;
 *      sending "-90\n" rewinds by 90 degrees.
 * Call AFTER UART3_Init() and AFTER FOC is up. */
void UART3_StartCmdRx(void);
int  UART3_GetByte(void);

#endif /* _QWR_UART_DRIVER_H_ */
