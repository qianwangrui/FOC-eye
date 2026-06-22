#include "qwr_FOC.h"
#include "debug_cpu.h"
#include "qwr_FOC_peri_init.h"     /* extern htim1 */
#include "qwr_MT6701_driver.h"     /* MT6701_GetAngleDeg */
#include "qwr_INA240_driver.h"     /* INA240_ReadCurrent_* */
#include <math.h>

#define SQRT3       1.7320508076f
#define INV_SQRT3   0.5773502692f
#define SQRT3_2     0.8660254038f   /* sqrt(3)/2 */
#define TWO_PI      6.283185307f
#define DEG_TO_RAD  0.01745329252f

FOC_State_t g_foc = {0};

/* 1 kHz low-pass on encoder-derived speed for telemetry / Kd damping. */
#define FOC_VEL_LPF_ALPHA  0.25f
static float s_vel_filt_deg_s = 0.0f;
static uint16_t s_vel_tick = 0;
static uint32_t s_torque_cmd_ms = 0U;

/* Wrap angle difference to [-180, +180] degrees for shortest-path control. */
static inline float wrap_180(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

float FOC_EncoderToCmdDeg(float enc_deg)
{
    return FOC_EncoderSignedDeg(enc_deg);
}

float FOC_EncoderSignedDeg(float enc_0_360)
{
    float s = enc_0_360;

    if (s > 180.0f) {
        s -= 360.0f;
    }
    return s;
}

float FOC_NormalizeTargetEnc(float target_deg)
{
    return wrap_180(target_deg);
}

float FOC_GetPosErrDeg(void)
{
    return wrap_180(g_foc.pos_ref_deg - FOC_EncoderSignedDeg(g_foc.theta_mech_deg));
}

/* When closed-loop ISR is off, keep encoder telemetry fresh for printf / hand moves. */
void FOC_PollEncoderWhenIdle(void)
{
    if (FOC_IsRunning()) {
        return;
    }
    float deg = MT6701_GetAngleDeg();

    g_foc.theta_mech_deg  = deg;
    g_foc.theta_mech_prev = deg;
    g_foc.vel_deg_s       = 0.0f;
}

/* ====================  Inverse transforms (existing)  ==================== */

/* Inverse Park transform: dq frame (rotor) -> alpha-beta frame (stator). */
void park_inv(float Vd, float Vq, float theta, float *Valpha, float *Vbeta)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *Valpha = Vd * c - Vq * s;
    *Vbeta  = Vd * s + Vq * c;
}

/* Inverse Clarke transform: alpha-beta -> three-phase uvw. */
void clarke_inv(float Valpha, float Vbeta, float *Vu, float *Vv, float *Vw)
{
    *Vu =        Valpha;
    *Vv = -0.5f * Valpha + SQRT3_2 * Vbeta;
    *Vw = -0.5f * Valpha - SQRT3_2 * Vbeta;
}

/* ====================  Forward transforms  ==================== */

void clarke_fwd(float iu, float iv, float *ialpha, float *ibeta)
{
    /* Amplitude-invariant Clarke, assuming iu+iv+iw=0 (Kirchhoff). */
    *ialpha = iu;
    *ibeta  = (iu + 2.0f * iv) * INV_SQRT3;
}

void park_fwd(float ialpha, float ibeta, float theta, float *id, float *iq)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *id =  ialpha * c + ibeta * s;
    *iq = -ialpha * s + ibeta * c;
}

/* ====================  PWM / SVPWM helpers  ==================== */

/* SVPWM zero-sequence injection (min-max / centred PWM).
 * Same line-to-line result as sector SVPWM; keeps Valpha/Vbeta unchanged. */
static inline void svpwm_center(float *Vu, float *Vv, float *Vw)
{
    float vmax = (*Vu > *Vv) ? (*Vu > *Vw ? *Vu : *Vw) : (*Vv > *Vw ? *Vv : *Vw);
    float vmin = (*Vu < *Vv) ? (*Vu < *Vw ? *Vu : *Vw) : (*Vv < *Vw ? *Vv : *Vw);
    float vcom = 0.5f * (vmax + vmin);
    *Vu -= vcom;
    *Vv -= vcom;
    *Vw -= vcom;
}

/* Normalised phase voltage [-1, +1] -> duty -> CCR. */
static inline uint32_t volt_to_ccr(float v)
{
    /* SVPWM centred duty: (v + 1) / 2  */
    float duty = 0.5f * (v + 1.0f);
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    return (uint32_t)(duty * FOC_PWM_PERIOD);
}

/* First-order IIR low-pass state for the dq-axis voltage commands.
 * Applied at FOC_CONTROL_FREQ_HZ (20 kHz) inside FOC_ClosedLoopUpdate,
 * after the PI controllers and before park_inv. */
static float s_Vd_f = 0.0f, s_Vq_f = 0.0f;

/* Reset the Vd/Vq low-pass filter state (call when motor output is
 * disabled or realigned, to avoid a startup transient). */
static inline void vdq_lpf_reset(void)
{
    s_Vd_f = 0.0f;
    s_Vq_f = 0.0f;
}

/* Write Vu/Vv/Vw to PWM and save everything to g_foc telemetry. */
static void foc_write_output(float theta_elec, float Vd, float Vq,
                             float Valpha, float Vbeta,
                             float Vu, float Vv, float Vw)
{
    svpwm_center(&Vu, &Vv, &Vw);

    TIM1->CCR1 = volt_to_ccr(Vu);
    TIM1->CCR2 = volt_to_ccr(Vv);
    TIM1->CCR3 = volt_to_ccr(Vw);

    g_foc.theta_elec = theta_elec;
    g_foc.Vd = Vd;         g_foc.Vq = Vq;
    g_foc.Valpha = Valpha; g_foc.Vbeta = Vbeta;
    g_foc.Vu = Vu; g_foc.Vv = Vv; g_foc.Vw = Vw;
}

/* Wrap an angle in radians to [0, 2*pi). */
static inline float wrap_2pi(float a)
{
    while (a >= TWO_PI) a -= TWO_PI;
    while (a <  0.0f)   a += TWO_PI;
    return a;
}

/* ====================  PI controller  ==================== */

void PI_Init(PI_t *pi, float Kp, float Ki, float out_max)
{
    pi->Kp       = Kp;
    pi->Ki       = Ki;
    pi->integral = 0.0f;
    pi->out_max  = out_max;
}

void PI_Reset(PI_t *pi)
{
    pi->integral = 0.0f;
}

float PI_Update(PI_t *pi, float error, float dt)
{
    /* Accumulate integral with pre-clamp (anti-windup).
     * Keep Ki*integral within the output limit so a saturated controller
     * does not keep piling up integral and causing huge overshoot. */
    pi->integral += error * dt;

    if (pi->Ki > 1e-9f) {
        float i_limit = pi->out_max / pi->Ki;
        if (pi->integral >  i_limit) pi->integral =  i_limit;
        if (pi->integral < -i_limit) pi->integral = -i_limit;
    }

    float out = pi->Kp * error + pi->Ki * pi->integral;
    if (out >  pi->out_max) out =  pi->out_max;
    if (out < -pi->out_max) out = -pi->out_max;
    return out;
}

/* ====================  Open-loop (kept)  ==================== */

void FOC_OpenLoopUpdate(float theta_elec, float Vd, float Vq)
{
    float Valpha, Vbeta;
    float Vu, Vv, Vw;

    park_inv(Vd, Vq, theta_elec, &Valpha, &Vbeta);
    clarke_inv(Valpha, Vbeta, &Vu, &Vv, &Vw);

    foc_write_output(theta_elec, Vd, Vq, Valpha, Vbeta, Vu, Vv, Vw);
}

/* ====================  Closed-loop core  ==================== */

void FOC_Init(void)
{
    g_foc.theta_elec   = 0.0f;
    g_foc.theta_offset = 0.0f;
    g_foc.id_ref       = 0.0f;
    g_foc.iq_ref       = 0.0f;
    g_foc.aligned      = 0;

    PI_Init(&g_foc.pi_d, MOTOR_PI_D_KP, MOTOR_PI_D_KI, MOTOR_PI_D_OUT_MAX);
    PI_Init(&g_foc.pi_q, MOTOR_PI_Q_KP, MOTOR_PI_Q_KI, MOTOR_PI_Q_OUT_MAX);
}

float FOC_UpdateElectricalAngle(void)
{
    float mech_rad = MT6701_GetAngleDeg() * DEG_TO_RAD;
    /* Electrical angle = mechanical * pole_pairs * direction_sign,
     * then subtract the zero offset captured during alignment, then wrap. */
    float e = (float)FOC_ENC_DIR * mech_rad * (float)FOC_POLE_PAIRS
              - g_foc.theta_offset;
    g_foc.theta_elec = wrap_2pi(e);
    return g_foc.theta_elec;
}

void FOC_AlignRotor(void)
{
    /* Two-stage alignment, robust against cogging detents.
     *
     *   Stage 1: slowly rotate the stator field through a full electrical
     *            cycle with Vd pulling. The rotor follows the field and
     *            is forced out of any starting detent.
     *   Stage 2: hold the field at theta_elec=0 until the rotor settles,
     *            then capture the encoder reading as theta_offset so that
     *            subsequent encoder samples map theta_elec = 0 here.
     *
     * With a single-shot alignment (Stage 2 only) a rotor caught near a
     * 90-degree cogging peak can stay there, making theta_offset wrong by
     * ~pi/2 and producing zero net torque in closed loop ("twitch-in-place"
     * symptom). The sweep guarantees escape from any such starting point. */

    const uint32_t SWEEP_STEPS = 600;
    const uint32_t STEP_MS     = 3;          /* slow enough for rotor to follow */

    /* Start from a clean filter state so the first PWM write is not biased. */
    vdq_lpf_reset();

    /* Stage 1: sweep from 2*pi down to 0, so final position is at 0. */
    for (uint32_t i = 0; i <= SWEEP_STEPS; ++i) {
        float t = TWO_PI * (float)(SWEEP_STEPS - i) / (float)SWEEP_STEPS;
        FOC_OpenLoopUpdate(t, FOC_ALIGN_VD, 0.0f);
        HAL_Delay(STEP_MS);
    }

    /* Stage 2: hold at theta=0 to let rotor settle. */
    const uint32_t t_end = HAL_GetTick() + FOC_ALIGN_TIME_MS;
    while ((int32_t)(HAL_GetTick() - t_end) < 0) {
        FOC_OpenLoopUpdate(0.0f, FOC_ALIGN_VD, 0.0f);
    }

    /* Capture encoder -> electrical offset. Same direction sign as in
     * FOC_UpdateElectricalAngle so theta_elec evaluates to 0 here. */
    float mech_rad = MT6701_GetAngleDeg() * DEG_TO_RAD;
    g_foc.theta_offset = wrap_2pi((float)FOC_ENC_DIR * mech_rad
                                   * (float)FOC_POLE_PAIRS);
    g_foc.aligned = 1;

    /* Zero output (50% duty all phases = no differential). */
    FOC_OpenLoopUpdate(0.0f, 0.0f, 0.0f);

    /* Clear integrators so closed-loop starts clean. */
    PI_Reset(&g_foc.pi_d);
    PI_Reset(&g_foc.pi_q);
}

void FOC_SetCalibratedOffset(float theta_offset_rad)
{
    g_foc.theta_offset = wrap_2pi(theta_offset_rad);
    g_foc.aligned      = 1;

    /* Make sure PWM is at neutral and integrators are clean before the
     * closed-loop ISR is enabled. */
    FOC_OpenLoopUpdate(0.0f, 0.0f, 0.0f);
    PI_Reset(&g_foc.pi_d);
    PI_Reset(&g_foc.pi_q);
}

void FOC_ClosedLoopUpdate(float id_ref, float iq_ref, float dt)
{
    if (!g_foc.aligned) {
        /* Safe fallback: refuse to run closed-loop before alignment. */
        FOC_OpenLoopUpdate(0.0f, 0.0f, 0.0f);
        return;
    }

    g_foc.id_ref = id_ref;
    g_foc.iq_ref = iq_ref;

    /* 1. Update electrical angle from encoder. */
    float theta = FOC_UpdateElectricalAngle();

    /* 2. Read phase currents. iw via Kirchhoff. */
    float iu = INA240_ReadCurrent_IU();
    float iv = INA240_ReadCurrent_IV();
    float iw = -(iu + iv);
    g_foc.iu = iu; g_foc.iv = iv; g_foc.iw = iw;

    /* 3. Clarke + Park -> dq currents. */
    float ialpha, ibeta;
    clarke_fwd(iu, iv, &ialpha, &ibeta);
    g_foc.ialpha = ialpha; g_foc.ibeta = ibeta;

    float id, iq;
    park_fwd(ialpha, ibeta, theta, &id, &iq);
    g_foc.id = id; g_foc.iq = iq;

    /* 4. PI controllers. */
    float Vd = PI_Update(&g_foc.pi_d, id_ref - id, dt);
    float Vq = PI_Update(&g_foc.pi_q, iq_ref - iq, dt);

    /* 5. Saturate |(Vd, Vq)| <= FOC_V_MAX (SVPWM linear region). */
    float mag2 = Vd * Vd + Vq * Vq;
    float lim2 = FOC_V_MAX * FOC_V_MAX;
    if (mag2 > lim2) {
        float scale = FOC_V_MAX / sqrtf(mag2);
        Vd *= scale;
        Vq *= scale;
        /* Also clip integrator so we don't wind up further. */
        g_foc.pi_d.integral *= scale;
        g_foc.pi_q.integral *= scale;
    }

    /* 5b. One-pole low-pass on Vd/Vq to smooth the voltage command.
     * Done in the dq frame so it does not rotate the applied voltage
     * vector's electrical angle. See FOC_VDQ_LPF_ALPHA in qwr_FOC.h. */
    {
        const float a = FOC_VDQ_LPF_ALPHA;
        s_Vd_f += a * (Vd - s_Vd_f);
        s_Vq_f += a * (Vq - s_Vq_f);
        Vd = s_Vd_f;
        Vq = s_Vq_f;
    }

    /* 6. Inverse Park + inverse Clarke + SVPWM -> PWM. */
    float Valpha, Vbeta, Vu, Vv, Vw;
    park_inv(Vd, Vq, theta, &Valpha, &Vbeta);
    clarke_inv(Valpha, Vbeta, &Vu, &Vv, &Vw);

    foc_write_output(theta, Vd, Vq, Valpha, Vbeta, Vu, Vv, Vw);
}

/* ====================  Timer interrupt control  ==================== */

void FOC_UpdateMechanicalVelocity(void)
{
    float deg = MT6701_GetAngleDeg();
    float d_angle = wrap_180(deg - g_foc.theta_mech_prev);

    g_foc.theta_mech_deg  = deg;
    g_foc.theta_mech_prev = deg;

    float vel_raw = d_angle / FOC_VEL_DT;
    s_vel_filt_deg_s += FOC_VEL_LPF_ALPHA * (vel_raw - s_vel_filt_deg_s);
    g_foc.vel_deg_s = s_vel_filt_deg_s;
}

float FOC_GetMechanicalVelocityDegS(void)
{
    return g_foc.vel_deg_s;
}

float FOC_GetMechanicalVelocityRps(void)
{
    return g_foc.vel_deg_s / 360.0f;
}

float FOC_GetMechanicalVelocityRpm(void)
{
    return FOC_GetMechanicalVelocityRps() * 60.0f;
}

void FOC_StartClosedLoopISR(void)
{
    PI_Reset(&g_foc.pi_d);
    PI_Reset(&g_foc.pi_q);
    PI_Reset(&g_foc.pi_pos);
    g_foc.pos_tick = 0;
    s_vel_tick     = 0;
    s_vel_filt_deg_s = 0.0f;
    g_foc.theta_mech_deg  = MT6701_GetAngleDeg();
    g_foc.theta_mech_prev = g_foc.theta_mech_deg;
    g_foc.vel_deg_s       = 0.0f;
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
}

void FOC_StopClosedLoopISR(void)
{
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
    g_foc.torque_armed = 0U;
    FOC_OpenLoopUpdate(0.0f, 0.0f, 0.0f);
}

uint8_t FOC_IsRunning(void)
{
    return (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE) == SET) ? 1U : 0U;
}

static void foc_enter_idle(void)
{
    g_foc.torque_armed = 0U;
    g_foc.id_ref       = 0.0f;
    g_foc.iq_ref       = 0.0f;
    PI_Reset(&g_foc.pi_pos);
    PI_Reset(&g_foc.pi_d);
    PI_Reset(&g_foc.pi_q);
    FOC_OpenLoopUpdate(0.0f, 0.0f, 0.0f);
    __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
}

void FOC_InitPositionMode(float pos_Kp, float pos_Ki, float pos_Kd, float iq_max)
{
    if (FOC_IsRunning()) {
        FOC_StopClosedLoopISR();
    }
    FOC_EnablePositionMode(pos_Kp, pos_Ki, pos_Kd, iq_max);
    g_foc.torque_armed = 0U;
    g_foc.pos_ref_deg  = FOC_EncoderSignedDeg(g_foc.theta_mech_deg);
    FOC_OpenLoopUpdate(0.0f, 0.0f, 0.0f);
}

void FOC_RequestAngle(float pos_ref_deg)
{
    if (!g_foc.aligned) {
        return;
    }
    if (!g_foc.pos_mode) {
        FOC_EnablePositionMode(MOTOR_POS_KP, MOTOR_POS_KI, MOTOR_POS_KD, MOTOR_IQ_MAX);
    }

    g_foc.pos_ref_deg  = wrap_180(pos_ref_deg);
    s_torque_cmd_ms    = HAL_GetTick();
    g_foc.torque_armed = 1U;
    PI_Reset(&g_foc.pi_pos);

    if (!FOC_IsRunning()) {
        float deg = MT6701_GetAngleDeg();
        g_foc.theta_mech_deg  = deg;
        g_foc.theta_mech_prev = deg;
        g_foc.vel_deg_s       = 0.0f;
        FOC_StartClosedLoopISR();
    }
}

void FOC_DisarmTorque(void)
{
    foc_enter_idle();
}

uint8_t FOC_IsTorqueArmed(void)
{
    return g_foc.torque_armed;
}

void FOC_PollTorqueCmdTimeout(void)
{
    if (!g_foc.torque_armed) {
        return;
    }
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_torque_cmd_ms) >= (uint32_t)MOTOR_TORQUE_CMD_TIMEOUT_MS) {
        foc_enter_idle();
    }
}

void FOC_EnablePositionMode(float pos_Kp, float pos_Ki, float pos_Kd, float iq_max)
{
    PI_Init(&g_foc.pi_pos, pos_Kp, pos_Ki, iq_max);
    g_foc.theta_mech_deg  = MT6701_GetAngleDeg();
    g_foc.theta_mech_prev = g_foc.theta_mech_deg;
    g_foc.vel_deg_s       = 0.0f;
    g_foc.pos_Kd          = pos_Kd;
    g_foc.pos_ref_deg     = FOC_EncoderSignedDeg(g_foc.theta_mech_deg);
    g_foc.pos_tick        = 0;
    g_foc.pos_mode        = 1;
}

void TIM1_UP_TIM16_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) &&
        __HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);

        debug_cpu_busy();

        /* Encoder speed + optional position loop at 1 kHz. */
        if (++s_vel_tick >= FOC_VEL_DECIMATION) {
            s_vel_tick = 0;
            FOC_UpdateMechanicalVelocity();
        }

        if (!g_foc.torque_armed) {
            FOC_OpenLoopUpdate(0.0f, 0.0f, 0.0f);
        } else if (g_foc.pos_mode) {
            if (++g_foc.pos_tick >= FOC_POS_DECIMATION) {
                g_foc.pos_tick = 0;

                float err = FOC_GetPosErrDeg();

#if 0  /* position deadband disabled */
                if (fabsf(err) <= MOTOR_POS_DEADBAND_DEG
                    && fabsf(g_foc.vel_deg_s) <= MOTOR_POS_DEADBAND_VEL_DEGS) {
                    foc_enter_idle();
                } else
#endif
                {
                    float out = PI_Update(&g_foc.pi_pos, err, FOC_POS_DT)
                                - g_foc.pos_Kd * g_foc.vel_deg_s;

                    float lim = g_foc.pi_pos.out_max;
                    if (out >  lim) out =  lim;
                    if (out < -lim) out = -lim;

                    g_foc.iq_ref = out;
                    g_foc.id_ref = 0.0f;
                    FOC_ClosedLoopUpdate(g_foc.id_ref, g_foc.iq_ref, FOC_CONTROL_DT);
                }
            } else {
                FOC_ClosedLoopUpdate(g_foc.id_ref, g_foc.iq_ref, FOC_CONTROL_DT);
            }
        } else {
            FOC_ClosedLoopUpdate(g_foc.id_ref, g_foc.iq_ref, FOC_CONTROL_DT);
        }

        debug_cpu_idle();
    }
}
