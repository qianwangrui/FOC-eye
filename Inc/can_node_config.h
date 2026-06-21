#ifndef CAN_NODE_CONFIG_H
#define CAN_NODE_CONFIG_H

/* =============================================================================
 * 6 节点菊花链 CAN：每块板烧录前只改下面两项。
 *
 * 节点 0（网关）：CAN_NODE_ID=0, CAN_NODE_IS_GATEWAY=1
 *   - USART3 PB11 / USART1 PB7 接收 8 字节批量包（见 tools/gateway_send_angles.py）
 *   - CAN 发 1 帧广播 6 路 int8 角度（1° 精度），本机控制电机 0
 *
 * 节点 1..5（从站）：CAN_NODE_ID=1..5, CAN_NODE_IS_GATEWAY=0
 *   - 监听 CAN ID 0x300，取 payload[2+NODE_ID] 作为本机角度
 *
 * UART / CAN 载荷（8 字节）：
 *   [0xC6, seq, ang0..ang5]  — UART，6 x int8 度
 *   [seq, 0x01, ang0..ang5]  — CAN ID 0x300，6 x int8 度
 * ============================================================================= */

#define CAN_MOTOR_COUNT       1U

/* 本板电机编号：0 = 网关板（第 1 个垫片），1..5 = 从站 */
#define CAN_NODE_ID           0U

/* 1 = 网关（接收 USART3 外部命令并 CAN 转发）；0 = 从站 */
#define CAN_NODE_IS_GATEWAY   1U

#if (CAN_NODE_ID >= CAN_MOTOR_COUNT)
#error "CAN_NODE_ID must be 0 .. CAN_MOTOR_COUNT-1"
#endif

#if CAN_NODE_IS_GATEWAY && (CAN_NODE_ID != 0U)
#warning "Gateway node usually has CAN_NODE_ID 0"
#endif

#endif /* CAN_NODE_CONFIG_H */
