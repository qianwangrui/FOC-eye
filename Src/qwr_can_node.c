#include "qwr_can_node.h"
#include "can_node_config.h"
#include "qwr_fdcan_driver.h"
#include "qwr_FOC.h"
#include "main.h"
#include <string.h>

CAN_NodeStats_t g_can_node = {0};

#define CAN_ANGLE_BATCH_ID      0x300U
#define UART_BATCH_MAGIC        0xC6U
#define UART_BATCH_LEN          (2U + CAN_MOTOR_COUNT)
#define UART_BATCH_TIMEOUT_MS   50U

static int8_t s_angles_deg[CAN_MOTOR_COUNT];
static uint8_t s_uart_rx_buf[UART_BATCH_LEN];
static uint8_t s_uart_rx_idx = 0U;
static uint32_t s_uart_last_byte_ms = 0U;

static void uart_batch_parser_reset(void)
{
    s_uart_rx_idx = 0U;
}

static float i8_to_deg(int8_t ang)
{
    return (float)ang;
}

static void pack_can_batch(uint8_t *out, uint8_t seq, const int8_t *angles)
{
    out[0] = seq;
    out[1] = 0x01U;
    for (uint8_t i = 0U; i < CAN_MOTOR_COUNT; i++) {
        out[2U + i] = (uint8_t)angles[i];
    }
}

static uint8_t parse_can_batch(const uint8_t *in, uint8_t rx_len, uint8_t *seq, int8_t *angles_out)
{
    if (rx_len < 2U + CAN_MOTOR_COUNT || in[1] != 0x01U) {
        return 0U;
    }
    if (seq) {
        *seq = in[0];
    }
    if (angles_out) {
        for (uint8_t i = 0U; i < CAN_MOTOR_COUNT; i++) {
            angles_out[i] = (int8_t)in[2U + i];
        }
    }
    return 1U;
}

static void apply_local_angle(float deg, uint8_t seq)
{
    g_foc.pos_ref_deg = deg;
    g_can_node.last_angle_deg = deg;
    g_can_node.last_seq       = seq;
    g_can_node.angle_rx_cnt++;
    FOC_RequestAngle(deg);
}

static void on_can_batch(uint8_t seq, const int8_t *angles)
{
    apply_local_angle(i8_to_deg(angles[CAN_NODE_ID]), seq);
}

static uint8_t can_send_batch(uint8_t seq, const int8_t *angles)
{
    uint8_t payload[8];

    pack_can_batch(payload, seq, angles);
    if (FDCAN_SendStdWait(CAN_ANGLE_BATCH_ID, payload, sizeof(payload), 5U)) {
        g_can_node.tx_cnt++;
        return 1U;
    }
    g_can_node.tx_fail++;
    return 0U;
}

static void forward_uart_batch(uint8_t seq)
{
    if (can_send_batch(seq, s_angles_deg)) {
        /* TX node does not loop back into its own RX FIFO — apply locally here. */
        on_can_batch(seq, s_angles_deg);
    }
    g_can_node.batch_cnt++;
}

static void on_uart_batch(const uint8_t *buf)
{
    uint8_t seq = buf[1];

    for (uint8_t i = 0U; i < CAN_MOTOR_COUNT; i++) {
        s_angles_deg[i] = (int8_t)buf[2U + i];
    }
    forward_uart_batch(seq);
}

static void poll_can_rx(void)
{
    uint8_t payload[8];
    uint32_t rx_id = 0U;
    uint8_t rx_len = 0U;
    int8_t angles[CAN_MOTOR_COUNT];
    uint8_t seq = 0U;

    while (FDCAN_TryRecvStd(&rx_id, payload, &rx_len)) {
        g_can_node.rx_cnt++;
        if (rx_id != CAN_ANGLE_BATCH_ID) {
            continue;
        }
        if (!parse_can_batch(payload, rx_len, &seq, angles)) {
            continue;
        }
        on_can_batch(seq, angles);
    }
}

void CAN_ForwardAngles(const int8_t angles_deg[CAN_MOTOR_COUNT], uint8_t seq)
{
    uart_batch_parser_reset();
    if (angles_deg) {
        for (uint8_t i = 0U; i < CAN_MOTOR_COUNT; i++) {
            s_angles_deg[i] = angles_deg[i];
        }
        forward_uart_batch(seq);
    }
}

void CAN_GatewayForwardAngles(const int8_t angles_deg[CAN_MOTOR_COUNT], uint8_t seq)
{
    CAN_ForwardAngles(angles_deg, seq);
}

uint8_t CAN_FeedUartByte(uint8_t byte)
{
    uint32_t now = HAL_GetTick();

    if (s_uart_rx_idx != 0U &&
        (int32_t)(now - s_uart_last_byte_ms) > (int32_t)UART_BATCH_TIMEOUT_MS) {
        uart_batch_parser_reset();
    }
    s_uart_last_byte_ms = now;

    if (byte == UART_BATCH_MAGIC) {
        s_uart_rx_buf[0] = byte;
        s_uart_rx_idx = 1U;
        return 1U;
    }

    if (s_uart_rx_idx == 0U) {
        return 0U;
    }

    s_uart_rx_buf[s_uart_rx_idx++] = byte;
    if (s_uart_rx_idx >= UART_BATCH_LEN) {
        on_uart_batch(s_uart_rx_buf);
        uart_batch_parser_reset();
    }
    return 1U;
}

uint8_t CAN_GatewayFeedUartByte(uint8_t byte)
{
    return CAN_FeedUartByte(byte);
}

void CAN_NodeInit(void)
{
    memset(&g_can_node, 0, sizeof(g_can_node));
    uart_batch_parser_reset();
    s_uart_last_byte_ms = 0U;
}

void CAN_NodePoll(void)
{
    poll_can_rx();
}
