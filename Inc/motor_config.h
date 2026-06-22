#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "can_node_config.h"

/* 每块板只改 can_node_config.h 里的 CAN_NODE_ID；极对数随节点自动选 profile。 */
#if CAN_NODE_ID == 0
#define MOTOR_PROFILE  1   /* node 0: 6 pole pairs */
#else
#define MOTOR_PROFILE  1   /* nodes 1..5: 6 pole pairs (eyelid) */
#endif

#if MOTOR_PROFILE == 0
/* ---------- 旧电机：7 极对数 gimbal ---------- */
#define MOTOR_POLE_PAIRS     7U
#define MOTOR_ENC_DIR        (+1)

#define MOTOR_PI_D_KP        1.0f
#define MOTOR_PI_D_KI        5.0f
#define MOTOR_PI_D_OUT_MAX   1.0f
#define MOTOR_PI_Q_KP        2.0f
#define MOTOR_PI_Q_KI        1.0f
#define MOTOR_PI_Q_OUT_MAX   1.0f

#define MOTOR_POS_KP         0.041f
#define MOTOR_POS_KI         0.010f
#define MOTOR_POS_KD         0.0002f
#define MOTOR_IQ_MAX         1.0f

#define MOTOR_CAL_OFFSET     0.09f
#define MOTOR_SKIP_ALIGN     0
#define MOTOR_ALIGN_ON_BOOT  0

#elif MOTOR_PROFILE == 1
/* ---------- 6 极对数电机 ---------- */
#define MOTOR_POLE_PAIRS     6U
#define MOTOR_ENC_DIR        (+1)

#define MOTOR_PI_D_KP        0.0f
#define MOTOR_PI_D_KI        0.0f
#define MOTOR_PI_D_OUT_MAX   0.5f
#define MOTOR_PI_Q_KP        0.8f
#define MOTOR_PI_Q_KI        1.0f
#define MOTOR_PI_Q_OUT_MAX   2.0f

#define MOTOR_POS_KP         0.07f
#define MOTOR_POS_KI         0.000f
#define MOTOR_POS_KD         0.0002f
#define MOTOR_IQ_MAX         200.0f

#define MOTOR_CAL_OFFSET     0.0f
#define MOTOR_SKIP_ALIGN     0
#define MOTOR_ALIGN_ON_BOOT  0

#else
#error "MOTOR_PROFILE must be 0 or 1"
#endif

#if 0
#define MOTOR_POS_DEADBAND_DEG      5.0f
#define MOTOR_POS_DEADBAND_VEL_DEGS 15.0f
#endif

#define MOTOR_TORQUE_CMD_TIMEOUT_MS  3000U

#define VOFA_TELEM_ENABLE            1
#define VOFA_TELEM_MS                10U

#endif /* MOTOR_CONFIG_H */
