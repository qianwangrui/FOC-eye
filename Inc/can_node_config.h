#ifndef CAN_NODE_CONFIG_H
#define CAN_NODE_CONFIG_H

/* =============================================================================
 * 6 节点菊花链 CAN：每块板烧录前只改 CAN_NODE_ID（0..5）。
 *
 * 所有节点能力相同：
 *   - PB11 (USART3) / PB7 (USART1) 收 8 字节批量包或文本命令
 *   - 收到 UART 批量包 → CAN 广播 + 本机应用 angles[CAN_NODE_ID]
 *   - 收到 CAN ID 0x300 → 本机应用 angles[CAN_NODE_ID]
 *
 * UART 批量（8 字节）：[0xC6, seq, ang0..ang5]
 * CAN 载荷（8 字节）：  [seq, 0x01, ang0..ang5]  ID 0x300
 * ============================================================================= */

#define CAN_MOTOR_COUNT  6U

/* 本板电机编号 0..5（对应 CAN 帧 payload[2+ID]） */
#define CAN_NODE_ID      0U

#if (CAN_NODE_ID >= CAN_MOTOR_COUNT)
#error "CAN_NODE_ID must be 0 .. CAN_MOTOR_COUNT-1"
#endif

#endif /* CAN_NODE_CONFIG_H */
