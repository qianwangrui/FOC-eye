#ifndef QWR_FDCAN_DRIVER_H
#define QWR_FDCAN_DRIVER_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* Classic CAN loopback test frame ID. */
#define FDCAN_LB_TEST_ID   0x123U

typedef struct {
    uint32_t tx_cnt;
    uint32_t rx_ok_cnt;
    uint32_t rx_fail_cnt;
    uint32_t last_rx_id;
    uint8_t  last_rx_data[8];
    uint8_t  last_rx_len;
} FDCAN_LoopbackStats_t;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_LoopbackStats_t g_fdcan_lb;

void FDCAN_Init(void);
void FDCAN_Start(void);
uint8_t FDCAN_SendStd(uint16_t std_id, const uint8_t *data, uint8_t len);
uint8_t FDCAN_TryRecvStd(uint32_t *std_id, uint8_t *data, uint8_t *len);
void FDCAN_LoopbackPoll(void);

#endif /* QWR_FDCAN_DRIVER_H */
