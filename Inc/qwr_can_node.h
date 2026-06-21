#ifndef QWR_CAN_NODE_H
#define QWR_CAN_NODE_H

#include <stdint.h>
#include "can_node_config.h"

typedef struct {
    uint32_t tx_cnt;
    uint32_t rx_cnt;
    uint32_t tx_fail;
    uint32_t batch_cnt;      /* gateway: UART batches forwarded */
    uint32_t angle_rx_cnt;   /* slave/gateway: angle frames applied */
    uint8_t  last_seq;
    float    last_angle_deg;
} CAN_NodeStats_t;

extern CAN_NodeStats_t g_can_node;

void CAN_NodeInit(void);
void CAN_NodePoll(void);

/* Gateway only: forward 6 motor angles (int8 deg) over CAN + apply local motor. */
void CAN_GatewayForwardAngles(const int8_t angles_deg[CAN_MOTOR_COUNT], uint8_t seq);

/* Gateway only: feed bytes from command UART RX (PB11 or PB7).
 * Returns 1 if byte belongs to the 0xC6 binary batch (do not parse as text). */
uint8_t CAN_GatewayFeedUartByte(uint8_t byte);

#endif /* QWR_CAN_NODE_H */
