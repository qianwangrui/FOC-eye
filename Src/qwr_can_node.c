#include "qwr_can_node.h"
#include "can_node_config.h"
#include "qwr_fdcan_driver.h"
#include "qwr_FOC.h"
#include "main.h"
#include <string.h>

CAN_NodeStats_t g_can_node = {0};

/* Classic CAN 500 kbps: one broadcast frame, 6 x int8 angles (1 deg). */
#define CAN_ANGLE_BATCH_ID      0x300U

#define UART_BATCH_MAGIC        0xC6U
#define UART_BATCH_LEN          (2U + CAN_MOTOR_COUNT)   /* magic, seq, 6 x int8 */
#define UART_BATCH_TIMEOUT_MS   50U

static int8_t s_angles_deg[CAN_MOTOR_COUNT];
static uint8_t s_uart_rx_buf[UART_BATCH_LEN];
static uint8_t s_uart_rx_idx = 0U;
static uint32_t s_uart_last_byte_ms = 0U;

static void gateway_uart_parser_reset(void)
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
    out[1] = 0x01U;   /* valid */
    for (uint8_t i = 0U; i < CAN_MOTOR_COUNT; i++) {
        out[2U + i] = (uint8_t)angles[i];
    }
}

#if !CAN_NODE_IS_GATEWAY
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
#endif

static void apply_local_angle(float deg, uint8_t seq)
{
    g_foc.pos_ref_deg = deg;
    g_can_node.last_angle_deg = deg;
    g_can_node.last_seq       = seq;
    g_can_node.angle_rx_cnt++;
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

static void gateway_forward_batch(uint8_t seq)
{
    if (can_send_batch(seq, s_angles_deg)) {
        /* Apply local motor only after the frame is on the bus (same as slave RX timing). */
        on_can_batch(seq, s_angles_deg);
    }
    g_can_node.batch_cnt++;
}

static void gateway_on_uart_batch(const uint8_t *buf)
{
    uint8_t seq = buf[1];

    for (uint8_t i = 0U; i < CAN_MOTOR_COUNT; i++) {
        s_angles_deg[i] = (int8_t)buf[2U + i];
    }
    gateway_forward_batch(seq);
}

#if !CAN_NODE_IS_GATEWAY
static void slave_poll_can(void)
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
#endif

#if CAN_NODE_IS_GATEWAY
static void gateway_poll_can(void)
{
    uint8_t payload[8];
    uint32_t rx_id = 0U;
    uint8_t rx_len = 0U;

    while (FDCAN_TryRecvStd(&rx_id, payload, &rx_len)) {
        g_can_node.rx_cnt++;
    }
}
#endif

void CAN_GatewayForwardAngles(const int8_t angles_deg[CAN_MOTOR_COUNT], uint8_t seq)
{
#if CAN_NODE_IS_GATEWAY
    gateway_uart_parser_reset();
    if (angles_deg) {
        for (uint8_t i = 0U; i < CAN_MOTOR_COUNT; i++) {
            s_angles_deg[i] = angles_deg[i];
        }
        gateway_forward_batch(seq);
    }
#else
    (void)angles_deg;
    (void)seq;
#endif
}

uint8_t CAN_GatewayFeedUartByte(uint8_t byte)
{
#if !CAN_NODE_IS_GATEWAY
    (void)byte;
    return 0U;
#else
    uint32_t now = HAL_GetTick();

    if (s_uart_rx_idx != 0U &&
        (int32_t)(now - s_uart_last_byte_ms) > (int32_t)UART_BATCH_TIMEOUT_MS) {
        gateway_uart_parser_reset();
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
        gateway_on_uart_batch(s_uart_rx_buf);
        gateway_uart_parser_reset();
    }
    return 1U;
#endif
}

void CAN_NodeInit(void)
{
    memset(&g_can_node, 0, sizeof(g_can_node));
    gateway_uart_parser_reset();
    s_uart_last_byte_ms = 0U;
}

void CAN_NodePoll(void)
{
#if CAN_NODE_IS_GATEWAY
    gateway_poll_can();
#else
    slave_poll_can();
#endif
}
