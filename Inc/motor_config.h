#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/* =============================================================================
 * 换电机：只改 MOTOR_PROFILE（0 = 旧 7PP，1 = 新 6PP），再按 VOFA+ 微调 PID。
 * ============================================================================= */
#define MOTOR_PROFILE  0

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

#define MOTOR_POS_KP         0.040f
#define MOTOR_POS_KI         0.010f
#define MOTOR_POS_KD         0.0002f
#define MOTOR_IQ_MAX         1.0f

#define MOTOR_CAL_OFFSET     0.09f   /* rad，仅 MOTOR_SKIP_ALIGN=1 时用 */
#define MOTOR_SKIP_ALIGN     0       /* 1 = 跳过 AlignRotor，用上面 offset */

#elif MOTOR_PROFILE == 1
/* ---------- 新电机：6 极对数（初值同旧电机，上板后按 VOFA+ 调） ---------- */
#define MOTOR_POLE_PAIRS     6U
#define MOTOR_ENC_DIR        (+1)    /* 闭环锁死/嗡嗡响就改成 -1 */

#define MOTOR_PI_D_KP        0.4f
#define MOTOR_PI_D_KI        2.0f
#define MOTOR_PI_D_OUT_MAX   0.5f
#define MOTOR_PI_Q_KP        1.0f
#define MOTOR_PI_Q_KI        0.0f
#define MOTOR_PI_Q_OUT_MAX   20.0f

#define MOTOR_POS_KP         0.080f
#define MOTOR_POS_KI         0.010f
#define MOTOR_POS_KD         0.0001f
#define MOTOR_IQ_MAX         2.0f

#define MOTOR_CAL_OFFSET     0.0f
#define MOTOR_SKIP_ALIGN     0       /* 换电机必须重新 AlignRotor */

#else
#error "MOTOR_PROFILE must be 0 or 1"
#endif

#endif /* MOTOR_CONFIG_H */
