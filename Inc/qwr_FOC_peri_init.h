#ifndef _QWR_FOC_PERI_INIT_H
#define _QWR_FOC_PERI_INIT_H

#include "stm32g4xx_hal.h"

/* ===== 正确的写法：.h 文件只放声明 ===== */
extern TIM_HandleTypeDef htim1;
void FOC_GPIO_Init(void);
void FOC_TIM1_PWM_Init(void);

/* ===== 以下是原来的代码（已注释掉），问题说明见每段注释 ===== */

#if 0 /* --- 原始代码开始 --- */

/* [问题1] 缺少 #include "stm32g4xx_hal.h" */
/* [问题2] .h 文件里不能写可执行语句，赋值和函数调用必须在函数体内 */

TIM_Base_InitTypeDef tim = {0};
tim.Prescaler         = 0;
tim.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
tim.Period            = 4250 - 1;
tim.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
tim.RepetitionCounter = 1;
tim.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

/* [问题3] 参数类型不对，应该传 TIM_HandleTypeDef*，不是 TIM_Base_InitTypeDef* */
HAL_TIM_Base_Init(&tim);

TIM_OC_InitTypeDef oc = {0};
oc.OCMode       = TIM_OCMODE_PWM1;
oc.Pulse        = 0;
oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);
HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2);
HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3);

/* [问题4] 缺少 GPIO 初始化、时钟使能、PWM Start */

#endif /* --- 原始代码结束 --- */

#endif /* _QWR_FOC_PERI_INIT_H */
