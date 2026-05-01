#include "qwr_FOC_peri_init.h"

TIM_HandleTypeDef htim1;


 void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_TIM1_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);
}


void FOC_GPIO_Init(void)
{
    /* TIM1 PWM 引脚(PA8/PA9/PA10)已在 HAL_TIM_PWM_MspInit 中初始化 */
    /* 此处可添加其他 GPIO 初始化，如 LED、使能脚等 */
}

void FOC_TIM1_PWM_Init(void)
{


    /* 时基配置 */
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period            = 4250 - 1;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);

    /* 通道配置 */
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = TIM_OCMODE_PWM1;
    oc.Pulse        = 2125;
    oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_3);

    /* 死区和刹车配置（来自 hal_tim_ex） */
    /* 如果你的硬件有互补输出（CHxN），取消下面的注释 */
#if 0
    TIM_BreakDeadTimeConfigTypeDef bdt = {0};
    bdt.OffStateRunMode  = TIM_OSSR_ENABLE;      // 运行模式下，关闭的通道输出由 IdleState 决定
    bdt.OffStateIDLEMode = TIM_OSSI_ENABLE;      // 空闲模式下同上
    bdt.LockLevel        = TIM_LOCKLEVEL_OFF;     // 不锁寄存器
    bdt.DeadTime         = 84;                    // 死区时间 ≈ 500ns（170MHz 下每个 tick ≈ 5.9ns，84 × 5.9 ≈ 500ns）
    bdt.BreakState       = TIM_BREAK_DISABLE;     // 暂不使能刹车，需要时改为 ENABLE
    bdt.BreakPolarity    = TIM_BREAKPOLARITY_HIGH; // 刹车信号高电平有效
    bdt.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE; // 刹车释放后自动恢复输出
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdt);
#endif

    /* 启动三路 PWM */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
}
