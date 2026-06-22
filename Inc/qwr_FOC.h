#ifndef _QWR_FOC_H_
#define _QWR_FOC_H_

#include "stm32g4xx_hal.h"
#include "motor_config.h"
#include <stdint.h>

/* PWM period from TIM1 init (ARR = 4250-1, so 4250 ticks per half cycle). */
#define FOC_PWM_PERIOD   4250U

#define FOC_POLE_PAIRS   MOTOR_POLE_PAIRS
#define FOC_ENC_DIR      MOTOR_ENC_DIR

/* Voltage magnitude limit for the dq voltage vector (normalised, [0, 1]).
 * SVPWM linear region: sqrt(Vd^2+Vq^2) <= ~1.0 (inscribed circle in hexagon).
 * We leave some headroom below 1.0. */
#define FOC_V_MAX        0.95f

/* Rotor alignment parameters (open-loop pre-pulse that locks rotor to a known
 * electrical angle, so we can capture the encoder zero offset). */
#define FOC_ALIGN_VD     0.40f    /* small d-axis voltage to pull rotor */
#define FOC_ALIGN_TIME_MS 4000U   /* hold long enough for rotor to settle */

/* Closed-loop control frequency when running from TIM1 update interrupt.
 * Center-aligned PWM with RCR=1 → interrupt once per PWM cycle ≈ 20 kHz. */
#define FOC_CONTROL_FREQ_HZ  20000U
#define FOC_CONTROL_DT       (1.0f / FOC_CONTROL_FREQ_HZ)

/* Position-loop decimation: runs every N current-loop ticks.
 * 20000 / 20 = 1000 Hz position loop. */
#define FOC_POS_DECIMATION   20U
#define FOC_POS_DT           (FOC_CONTROL_DT * FOC_POS_DECIMATION)

/* Mechanical velocity estimate: encoder delta at 1 kHz (same decimation). */
#define FOC_VEL_DECIMATION   FOC_POS_DECIMATION
#define FOC_VEL_DT           FOC_POS_DT

/* First-order IIR low-pass filter on the dq-axis voltage commands
 * (Vd, Vq), applied AFTER the PI controllers and BEFORE park_inv.
 * Runs at FOC_CONTROL_FREQ_HZ (20 kHz).
 *
 *   y[n] = y[n-1] + alpha * (x[n] - y[n-1])
 *
 * Filtering in the dq frame (not the UVW frame) is safe because:
 *   - Vd/Vq are DC in steady state, so the filter does not bend the
 *     electrical angle of the applied voltage vector.
 *   - Only the magnitude response of the current loop is affected;
 *     phase on the applied stator voltage is preserved.
 *
 * alpha in (0, 1]:
 *   1.0  = no filtering (pass-through)
 *   0.47 ≈ -3 dB cutoff around 2.0 kHz   (alpha = 1 - exp(-2*pi*fc/fs))
 *   0.27 ≈ -3 dB cutoff around 1.0 kHz
 *   0.03 ≈ -3 dB cutoff around 100 Hz    (heavy smoothing, detune pi_d/q) */
#define FOC_VDQ_LPF_ALPHA    1.0f

/* =========================  Data structures  ========================= */

/* Simple anti-windup PI controller (symmetric output clamp). */
typedef struct {
    float Kp;
    float Ki;
    float integral;      /* running integral of error*dt */
    float out_max;       /* symmetric output clamp, used for both |out| and
                          * pre-clamp on integral to avoid windup. */
} PI_t;

/* Snapshot of the latest FOC computation, for telemetry / VOFA+ plotting
 * and for the controller itself to remember state across iterations. */
typedef struct {
    /* Angles (electrical, radians, wrapped to [0, 2*pi)). */
    float theta_elec;        /* used by Park (inverse & forward) */
    float theta_offset;      /* encoder-to-electrical zero offset */

    /* Phase currents (A), after Clarke/Park. */
    float iu, iv, iw;
    float ialpha, ibeta;
    float id, iq;

    /* Current targets (A). */
    float id_ref, iq_ref;

    /* dq voltage command (normalised, [-1, +1]) and stator-frame result. */
    float Vd, Vq;
    float Valpha, Vbeta;
    float Vu, Vv, Vw;

    /* Controllers. */
    PI_t pi_d;
    PI_t pi_q;

    /* Position loop. */
    PI_t  pi_pos;
    float theta_mech_deg;    /* current mechanical angle (degrees, 0..360) */
    float theta_mech_prev;   /* previous sample, for velocity estimate */
    float vel_deg_s;         /* estimated velocity (deg/s) */
    float pos_ref_deg;       /* target angle in command space (signed deg, int8 protocol) */
    float pos_Kd;            /* velocity damping gain (A·s/deg) */
    uint8_t pos_mode;        /* 1 = position loop active, 0 = pure torque */
    uint16_t pos_tick;       /* decimation counter */
    uint8_t torque_armed;    /* 1 = apply torque (after angle command) */

    /* Whether FOC_AlignRotor has been called. If 0, closed-loop update
     * will refuse to run (theta_offset not set). */
    uint8_t aligned;
} FOC_State_t;

extern FOC_State_t g_foc;

/* =========================  Inverse transforms  ========================= */

/* Inverse Park: (Vd, Vq, theta) -> (Valpha, Vbeta) */
void park_inv(float Vd, float Vq, float theta, float *Valpha, float *Vbeta);

/* Inverse Clarke: (Valpha, Vbeta) -> (Vu, Vv, Vw) */
void clarke_inv(float Valpha, float Vbeta, float *Vu, float *Vv, float *Vw);

/* =========================  Forward transforms  ======================== */

/* Forward Clarke (amplitude invariant, assumes iu+iv+iw=0):
 *   ialpha = iu
 *   ibeta  = (iu + 2*iv) / sqrt(3) */
void clarke_fwd(float iu, float iv, float *ialpha, float *ibeta);

/* Forward Park: (ialpha, ibeta, theta) -> (id, iq)
 *   id =  ialpha*cos(t) + ibeta*sin(t)
 *   iq = -ialpha*sin(t) + ibeta*cos(t) */
void park_fwd(float ialpha, float ibeta, float theta, float *id, float *iq);

/* =========================  PI controller  ============================= */

void PI_Init  (PI_t *pi, float Kp, float Ki, float out_max);
void PI_Reset (PI_t *pi);
float PI_Update(PI_t *pi, float error, float dt);

/* =========================  FOC core API  ============================== */

/* Initialise FOC state and PI gains. Does NOT touch the motor. */
void FOC_Init(void);

/* Read MT6701 angle and convert to electrical radians, applying
 * theta_offset. Writes g_foc.theta_elec. Returns the new angle. */
float FOC_UpdateElectricalAngle(void);

/* Rotor alignment: drives Vd=FOC_ALIGN_VD, Vq=0, theta_elec=0 for
 * FOC_ALIGN_TIME_MS, then records encoder position as theta_offset so
 * subsequent encoder readings map to theta_elec correctly.
 * After return, outputs are zero and g_foc.aligned = 1. */
void FOC_AlignRotor(void);

/* Skip live alignment and use a previously calibrated electrical-angle
 * offset (radians, [0, 2*pi)). Stores it into g_foc.theta_offset and
 * sets g_foc.aligned = 1. Use this when the rotor + encoder mounting
 * is fixed and you have measured theta_offset once already. */
void FOC_SetCalibratedOffset(float theta_offset_rad);

/* One control cycle of closed-loop current control:
 *   1. Read encoder -> theta_elec
 *   2. Read phase currents iu, iv
 *   3. Clarke + Park -> id, iq
 *   4. PI(id, id_ref) -> Vd,  PI(iq, iq_ref) -> Vq
 *   5. Saturate Vdq to FOC_V_MAX circle
 *   6. Inverse Park + SVPWM -> TIM1 CCR
 * dt: elapsed seconds since last call (for integral). */
void FOC_ClosedLoopUpdate(float id_ref, float iq_ref, float dt);

/* One-shot open-loop update: given electrical angle and dq voltage command
 * (each in [-1, +1] = fraction of bus voltage), write the resulting PWM duty
 * to TIM1 CCR1/CCR2/CCR3. Kept for open-loop tests and by FOC_AlignRotor. */
void FOC_OpenLoopUpdate(float theta_elec, float Vd, float Vq);

/* Start running closed-loop control from TIM1 update interrupt.
 * Call after FOC_Init + FOC_AlignRotor.  Set g_foc.id_ref / iq_ref first. */
void FOC_StartClosedLoopISR(void);
void FOC_StopClosedLoopISR(void);
uint8_t FOC_IsRunning(void);

/* Enable position-loop mode.  After this call the ISR will run a 1 kHz
 * position PI whose output becomes iq_ref.  Set g_foc.pos_ref_deg to
 * the desired mechanical angle in degrees. */
void FOC_EnablePositionMode(float pos_Kp, float pos_Ki, float pos_Kd, float iq_max);

/* Configure position loop but leave motor idle (no PWM torque). */
void FOC_InitPositionMode(float pos_Kp, float pos_Ki, float pos_Kd, float iq_max);

/* Set target angle and run closed-loop until deadband, then auto-idle. */
void FOC_RequestAngle(float pos_ref_deg);

/* Force idle: zero PWM and stop control ISR. */
void FOC_DisarmTorque(void);

uint8_t FOC_IsTorqueArmed(void);

/* Call from main loop: disarm if no angle command for MOTOR_TORQUE_CMD_TIMEOUT_MS. */
void FOC_PollTorqueCmdTimeout(void);

/* MT6701 raw 0..360 → signed mechanical angle -180..+180. */
float FOC_EncoderSignedDeg(float enc_0_360);

/* Convert raw encoder degrees (0..360) to signed mechanical angle. */
float FOC_EncoderToCmdDeg(float enc_deg);

/* Wrap command/target angle to -180..+180. */
float FOC_NormalizeTargetEnc(float target_deg);

/* pos_ref target vs encoder, shortest path on circle. */
float FOC_GetPosErrDeg(void);

/* Refresh g_foc.theta_mech_deg from MT6701 when control ISR is idle. */
void FOC_PollEncoderWhenIdle(void);

/* Read MT6701, update g_foc.theta_mech_deg and g_foc.vel_deg_s (deg/s).
 * Called from the TIM1 ISR every FOC_VEL_DECIMATION ticks (1 kHz). */
void FOC_UpdateMechanicalVelocity(void);

/* Latest filtered mechanical speed from encoder differentiation. */
float FOC_GetMechanicalVelocityDegS(void);

/* Convenience: rev/s (圈/秒) = deg/s / 360. */
float FOC_GetMechanicalVelocityRps(void);

/* Convenience: RPM = rev/s * 60. */
float FOC_GetMechanicalVelocityRpm(void);

#endif /* _QWR_FOC_H_ */
