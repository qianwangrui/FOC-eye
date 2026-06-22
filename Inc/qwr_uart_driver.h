#ifndef _QWR_UART_DRIVER_H_
#define _QWR_UART_DRIVER_H_

#include "stm32g4xx_hal.h"

/* USART3 on PB10 (TX) / PB11 (RX), 115200 8N1. */
void UART3_Init(void);

/* USART1 on PB6 (TX) / PB7 (RX), 115200 8N1.
 * PB6: printf / VOFA+ telemetry out.
 * PB7: command RX (same as PB11). */
void UART1_Init(void);

/* Enable interrupt RX on PB11 (USART3) and PB7 (USART1), shared ring buffer.
 * Text commands (set/blink/pos) and gateway binary batch (0xC6) work on either pin.
 * Call AFTER UART init and AFTER FOC is up. */
void UART_StartCmdRx(void);
void UART3_StartCmdRx(void);   /* alias of UART_StartCmdRx() */
int  UART3_GetByte(void);
int  UART1_GetByte(void);      /* alias of UART3_GetByte() */

extern volatile uint32_t g_uart3_rx_cnt;
extern volatile uint32_t g_uart1_rx_cnt;

#endif /* _QWR_UART_DRIVER_H_ */
