#ifndef QWR_FDCAN_DRIVER_H
#define QWR_FDCAN_DRIVER_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* 0 = physical CAN bus (TCAN1042); 1 = MCU internal loopback only. */
#define FDCAN_USE_INTERNAL_LOOPBACK  0

/* Classic CAN heartbeat / test frame (500 kbps). */
#define FDCAN_LB_TEST_ID   0x234U

typedef struct {
    uint32_t tx_cnt;
    uint32_t rx_ok_cnt;   /* loopback: self-match; bus: frames received */
    uint32_t rx_fail_cnt; /* loopback: timeout; bus: TX fail */
    uint32_t last_rx_id;
    uint8_t  last_rx_data[8];
    uint8_t  last_rx_len;
} FDCAN_LoopbackStats_t;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_LoopbackStats_t g_fdcan_lb;

void FDCAN_Init(void);
void FDCAN_Start(void);
uint8_t FDCAN_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len);
/* Send then wait until the TX FIFO slot is free again (frame on bus). timeout_ms=0 -> no wait. */
uint8_t FDCAN_SendStdWait(uint16_t std_id, const uint8_t *data, uint8_t len, uint32_t timeout_ms);
uint8_t FDCAN_TryRecvStd(uint32_t *std_id, uint8_t *data, uint8_t *len);
void FDCAN_LoopbackPoll(void);
void FDCAN_BusPoll(void);

#endif /* QWR_FDCAN_DRIVER_H */
