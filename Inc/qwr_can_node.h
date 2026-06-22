#ifndef QWR_CAN_NODE_H
#define QWR_CAN_NODE_H

#include <stdint.h>
#include "can_node_config.h"

typedef struct {
    uint32_t tx_cnt;
    uint32_t rx_cnt;
    uint32_t tx_fail;
    uint32_t batch_cnt;      /* UART batches forwarded to CAN */
    uint32_t angle_rx_cnt;   /* angle commands applied locally */
    uint8_t  last_seq;
    float    last_angle_deg;
} CAN_NodeStats_t;

extern CAN_NodeStats_t g_can_node;

void CAN_NodeInit(void);
void CAN_NodePoll(void);

/* Forward 6 motor angles over CAN and apply local motor slot CAN_NODE_ID. */
void CAN_ForwardAngles(const int8_t angles_deg[CAN_MOTOR_COUNT], uint8_t seq);

/* Feed bytes from command UART RX (PB11 or PB7).
 * Returns 1 if byte belongs to the 0xC6 binary batch (skip text parser). */
uint8_t CAN_FeedUartByte(uint8_t byte);

/* Legacy names */
void CAN_GatewayForwardAngles(const int8_t angles_deg[CAN_MOTOR_COUNT], uint8_t seq);
uint8_t CAN_GatewayFeedUartByte(uint8_t byte);

#endif /* QWR_CAN_NODE_H */
