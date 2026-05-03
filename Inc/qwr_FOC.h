#ifndef _QWR_FOC_H_
#define _QWR_FOC_H_

#include "stm32g4xx_hal.h"

/* PWM period from TIM1 init (ARR = 4250-1, so 4250 ticks per half cycle). */
#define FOC_PWM_PERIOD   4250U

/* Snapshot of the latest FOC computation, for telemetry / VOFA+ plotting. */
typedef struct {
    float theta_elec;
    float Vd, Vq;
    float Valpha, Vbeta;
    float Vu, Vv, Vw;
} FOC_State_t;

extern FOC_State_t g_foc;

/* Inverse Park: (Vd, Vq, theta) -> (Valpha, Vbeta) */
void park_inv(float Vd, float Vq, float theta, float *Valpha, float *Vbeta);

/* Inverse Clarke: (Valpha, Vbeta) -> (Vu, Vv, Vw) */
void clarke_inv(float Valpha, float Vbeta, float *Vu, float *Vv, float *Vw);

/* One-shot open-loop update: given electrical angle and dq voltage command
 * (each in [-1, +1] = fraction of bus voltage), write the resulting PWM duty
 * to TIM1 CCR1/CCR2/CCR3. */
void FOC_OpenLoopUpdate(float theta_elec, float Vd, float Vq);

#endif /* _QWR_FOC_H_ */
