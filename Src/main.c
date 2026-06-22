/**
  ******************************************************************************
  * @file    Templates/Src/main.c
  * @author  MCD Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "qwr_FOC_peri_init.h"
#include "qwr_MT6701_driver.h"
#include "qwr_INA240_driver.h"
#include "qwr_uart_driver.h"
#include "qwr_fdcan_driver.h"
#include "qwr_can_node.h"
#include "can_node_config.h"
#include "qwr_FOC.h"
#include "motor_config.h"
#include "wink.h"
#include "debug_cpu.h"
#include "nvm_cal.h"


/** @addtogroup STM32G4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void){

  /* STM32G4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user
             can eventually implement his proper time base source (a general purpose
             timer for example or other time source), keeping in mind that Time base
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();
  debug_cpu_init();

  /* Configure the System clock to have a frequency of 170 MHz */
  SystemClock_Config();
  FOC_GPIO_Init();
  FOC_TIM1_PWM_Init();
  MT6701_SPI_Init();  /* HAL_SPI_MspInit 会在内部自动配置 GPIO */
  INA240_GPIO_Init();
  INA240_ADC_Init();
  UART3_Init();        /* RX: command channel from host        */
  UART1_Init();        /* TX: VOFA+ telemetry (printf goes here) */
  FDCAN_Init();        /* Classic CAN 500 kbps on PA11/12 */
  FDCAN_Start();
  CAN_NodeInit();
#if !VOFA_TELEM_ENABLE
  printf("can_node,id=%u,pp=%u,cmd=pb11+pb7,tx=pb6\n",
         (unsigned)CAN_NODE_ID, (unsigned)MOTOR_POLE_PAIRS);
#endif

  /* No boot banner: VOFA+ FireWater would try to parse it as data. */

  /* ====================================================================
   * PP CALIBRATION MODE
   * --------------------------------------------------------------------
   * When this is 1: motor is NOT driven (all phases 50% duty = 0V).
   * The loop just reads the encoder and prints:
   *   - mech_deg : raw mechanical angle from MT6701 (0..360)
   *   - theta_e  : electrical angle = mech_rad * FOC_POLE_PAIRS (0..2pi)
   *
   * Hand-rotate the rotor SLOWLY through ONE full mechanical revolution
   * and count how many times theta_e wraps from ~2*pi back to ~0:
   *
   *   wraps_per_mech_rev == real PP
   *
   * Compare to FOC_POLE_PAIRS to know if your setting is correct.
   * Set back to 0 when done. */
  
  //顺时针编码器读数变小，逆时针变大
  /* ---------- Closed-loop FOC bring-up sequence ---------- */
  FOC_Init();           /* gains, state defaults */

  {
    float theta_offset = 0.0f;
    uint8_t motor_ready = 0U;

    if (NVM_Cal_TryLoad(&theta_offset)) {
      FOC_SetCalibratedOffset(theta_offset);
      motor_ready = 1U;
      printf("# cal: loaded flash theta=%.6f rad\r\n", (double)theta_offset);
    } else if (MOTOR_SKIP_ALIGN) {
      FOC_SetCalibratedOffset(MOTOR_CAL_OFFSET);
      motor_ready = 1U;
      printf("# cal: skip align, use MOTOR_CAL_OFFSET\r\n");
    } else if (MOTOR_ALIGN_ON_BOOT) {
      FOC_AlignRotor();
      motor_ready = 1U;
      printf("# cal: MOTOR_ALIGN_ON_BOOT\r\n");
    } else {
      NVM_Cal_PrintLoadFail();
      printf("# cal: send 'cal align' (bare motor) then 'cal save'\r\n");
    }

    if (motor_ready) {
      APP_FOC_StartMotor();
    }
    printf("# boot: aligned=%u (flash cal %s)\r\n",
           (unsigned)g_foc.aligned,
           motor_ready ? "ok" : "missing");
  }

  /* Start bare-metal RXNE interrupt -> ring buffer for UART commands (PB11 + PB7). */
  UART_StartCmdRx();

  char    cmd_buf[80];
  uint8_t cmd_idx = 0;

  uint32_t next_can  = HAL_GetTick();
#if VOFA_TELEM_ENABLE
  uint32_t next_vofa = HAL_GetTick();
#endif

  /* Infinite loop — telemetry + command RX at 5 Hz (200 ms period).
   * Control loop runs independently in the TIM1 update ISR. */
  while (1)
  {
    /* ---- Drain RX ring buffer (PB11 USART3 + PB7 USART1) ---- */
    int b;
    while ((b = UART3_GetByte()) >= 0) {
      char c = (char)b;
      uint8_t consumed = CAN_FeedUartByte((uint8_t)b);
      if (consumed) {
        continue;
      }
      if (c == '\r' || c == '\n') {
        if (cmd_idx > 0) {
          cmd_buf[cmd_idx] = '\0';
          APP_RunCommand(cmd_buf);
        }
        cmd_idx = 0;
      } else if (cmd_idx < sizeof(cmd_buf) - 1) {
        cmd_buf[cmd_idx++] = c;
      }
    }

    /* ---- CAN RX/TX (every loop); stats print @ 2 Hz ---- */
    CAN_NodePoll();
    FOC_PollTorqueCmdTimeout();
    FOC_PollEncoderWhenIdle();

#if VOFA_TELEM_ENABLE
    {
      uint32_t now = HAL_GetTick();
      if ((int32_t)(now - next_vofa) >= 0) {
        next_vofa = now + VOFA_TELEM_MS;
        float enc = MT6701_GetAngleDeg();
        g_foc.theta_mech_deg = enc;
        if (!FOC_IsRunning()) {
          g_foc.theta_mech_prev = enc;
        }
        /* VOFA+ FireWater: enc_deg, target_deg, err_deg, iq_A */
        printf("%.3f,%.3f,%.3f,%.3f\n",
               (double)enc,
               (double)FOC_NormalizeTargetEnc(g_foc.pos_ref_deg),
               (double)FOC_GetPosErrDeg(),
               (double)g_foc.iq);
      }
    }
#else
    {
      uint32_t now = HAL_GetTick();
      if ((int32_t)(now - next_can) >= 0) {
        next_can = now + 500U;
        printf("can_node,id=%u,tx=%lu,rx=%lu,fail=%lu,batch=%lu,angle=%lu,seq=%u,"
               "pos=%.1f,enc=%.1f,err=%.1f,armed=%u,uart3=%lu,uart1=%lu\n",
               (unsigned)CAN_NODE_ID,
               (unsigned long)g_can_node.tx_cnt,
               (unsigned long)g_can_node.rx_cnt,
               (unsigned long)g_can_node.tx_fail,
               (unsigned long)g_can_node.batch_cnt,
               (unsigned long)g_can_node.angle_rx_cnt,
               (unsigned)g_can_node.last_seq,
               (double)FOC_NormalizeTargetEnc(g_foc.pos_ref_deg),
               (double)g_foc.theta_mech_deg,
               (double)FOC_GetPosErrDeg(),
               (unsigned)FOC_IsTorqueArmed(),
               (unsigned long)g_uart3_rx_cnt,
               (unsigned long)g_uart1_rx_cnt);
      }
    }
#endif
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 170000000
  *            HCLK(Hz)                       = 170000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 2
  *            PLL_N                          = 85
  *            PLL_P                          = 2
  *            PLL_Q                          = 2
  *            PLL_R                          = 2
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable voltage range 1 boost mode for frequency above 150 Mhz */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  __HAL_RCC_PWR_CLK_DISABLE();

  /* Activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM            = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN            = 85;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2; //qwr: this param control sys clock. sysclk=HSE/M*B/R=8000000/2*85/2=170000000
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType           = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | \
                                           RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */


