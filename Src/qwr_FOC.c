#include "qwr_FOC.h"
#include "qwr_FOC_peri_init.h"   /* extern htim1 */
#include <math.h>

#define SQRT3_2    0.8660254038f   /* sqrt(3)/2 */

FOC_State_t g_foc = {0};

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

/* Clamp a normalised voltage (expected in [-1, +1]) to [0, PERIOD] duty. */
static inline uint32_t volt_to_ccr(float v)
{
    /* SPWM centred modulation: duty = (v + 1) / 2  */
    float duty = 0.5f * (v + 1.0f);
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    return (uint32_t)(duty * FOC_PWM_PERIOD);
}

void FOC_OpenLoopUpdate(float theta_elec, float Vd, float Vq)
{
    float Valpha, Vbeta;
    float Vu, Vv, Vw;

    park_inv(Vd, Vq, theta_elec, &Valpha, &Vbeta);
    clarke_inv(Valpha, Vbeta, &Vu, &Vv, &Vw);

    /* Write CCR registers directly: TIM1 is already running via HAL_TIM_PWM_Start. */
    TIM1->CCR1 = volt_to_ccr(Vu);
    TIM1->CCR2 = volt_to_ccr(Vv);
    TIM1->CCR3 = volt_to_ccr(Vw);

    /* Snapshot for telemetry. */
    g_foc.theta_elec = theta_elec;
    g_foc.Vd = Vd;       g_foc.Vq = Vq;
    g_foc.Valpha = Valpha; g_foc.Vbeta = Vbeta;
    g_foc.Vu = Vu; g_foc.Vv = Vv; g_foc.Vw = Vw;
}
