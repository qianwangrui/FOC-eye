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
#include "qwr_FOC_peri_init.h"
#include "qwr_MT6701_driver.h"
#include "qwr_INA240_driver.h"
#include "qwr_uart_driver.h"
#include "qwr_FOC.h"


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
int main(void)
{

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

  /* Configure the System clock to have a frequency of 170 MHz */
  SystemClock_Config();
  FOC_GPIO_Init();
  FOC_TIM1_PWM_Init();
  MT6701_SPI_Init();  /* HAL_SPI_MspInit 会在内部自动配置 GPIO */
  INA240_GPIO_Init();
  INA240_ADC_Init();
  UART1_Init();

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
  #define PP_CALIBRATION_MODE 0

  if (PP_CALIBRATION_MODE) {
      /* Park PWM at 50% duty so motor is freewheeling. */
      FOC_OpenLoopUpdate(0.0f, 0.0f, 0.0f);

      /* Use FOC_POLE_PAIRS just for the live theta_e plot; the wrap counter
       * itself is independent of PP and uses raw mech_deg crossings. */
      float    last_mech_deg     = MT6701_GetAngleDeg();
      float    last_theta_pp1    = last_mech_deg * (3.14159265f / 180.0f);
      uint32_t theta_e_wraps     = 0;       /* counted within current mech rev */
      uint32_t mech_revs         = 0;
      uint32_t loop_n            = 0;

      printf("# PP CAL: rotate the rotor SLOWLY one full turn.\n");
      printf("# Each completed mech revolution prints PP_measured.\n");

      while (1) {
          float mech_deg = MT6701_GetAngleDeg();

          /* Detect mech_deg sawtooth wrap (e.g. 359 -> 1). The direction
           * of rotation does not matter, we look at large jumps. */
          float mech_jump = mech_deg - last_mech_deg;
          if (mech_jump >  300.0f || mech_jump < -300.0f) {
              /* One full mechanical revolution completed in some direction. */
              mech_revs++;
              printf("# rev %lu: theta_e wraps observed = %lu  =>  PP = %lu\n",
                     (unsigned long)mech_revs,
                     (unsigned long)theta_e_wraps,
                     (unsigned long)theta_e_wraps);
              theta_e_wraps = 0;
          }
          last_mech_deg = mech_deg;

          /* Use REAL PP=1 for the wrap-counter so it counts encoder
           * sawtooths directly (theta_pp1 wraps == mech_deg wraps).
           * For the wrap counter that tells us PP, we instead count
           * how many times theta_with_PP1 * (some test factor) wraps —
           * but simpler: count theta_e wraps with PP_assumed = a known
           * factor. We use 1 here because then theta_e_wraps per mech rev
           * equals 1 (sanity); to actually MEASURE PP we count electrical
           * cycles directly via the unwrapped mech angle. See below. */

          /* Independent measurement: track mech_deg in radians without
           * wrapping (cumulative), and count how many times mech_rad
           * crosses an integer multiple of 2*pi/PP_TEST. We pick PP_TEST=1
           * (so it counts mech_revs, useless) — better approach:
           * count theta_e (with PP=1) wraps per mech rev, which is 1.
           *
           * Cleaner: just print mech_deg AND a counter that increments
           * every time mech_deg passes a fixed set of probe angles. Skip
           * all that complexity — instead just print:
           *   - mech_deg (so user can see one full rev was completed)
           *   - mech_rad * PP_assumed mod 2*pi  (wraps PP times per rev)
           * and auto-count wraps of channel 1.   */
          float theta_pp1 = mech_deg * (3.14159265f / 180.0f);
          /* Compute theta_e using the CURRENT FOC_POLE_PAIRS just for a
           * live wrap counter against the assumed PP. */
          float theta_e_assumed = theta_pp1 * (float)FOC_POLE_PAIRS;
          while (theta_e_assumed >= 6.283185f) theta_e_assumed -= 6.283185f;
          while (theta_e_assumed < 0.0f)       theta_e_assumed += 6.283185f;

          /* Detect theta_e wrap: was high, now low (or vice versa) - a
           * jump > pi in either direction is a wrap. */
          float te_jump = theta_e_assumed - last_theta_pp1;
          if (te_jump >  3.14159f || te_jump < -3.14159f) {
              theta_e_wraps++;
          }
          last_theta_pp1 = theta_e_assumed;

          /* Heartbeat once per second so you know the firmware is alive
           * and can see current angle. Wrap-event prints come in addition. */
          if (++loop_n >= 50) {
              loop_n = 0;
              printf("# mech=%6.1f deg   theta_e_norm=%.3f   wraps_so_far=%lu\n",
                     (double)mech_deg,
                     (double)(theta_e_assumed / 6.283185f),
                     (unsigned long)theta_e_wraps);
          }
          HAL_Delay(20);
      }
      /* Never reach here. */
  }

  /* ---------- Closed-loop FOC bring-up sequence ---------- */
  FOC_Init();           /* gains, state defaults */
  FOC_AlignRotor();     /* sweep + hold to record encoder zero offset */

  /* ---------- Sanity-check open-loop spin (debug) ----------
   * Spin the field very slowly for ~10 seconds at Vq=0.3 with no encoder
   * feedback. Watch the rotor and count how many times it goes around
   * during this window: that gives you POLE PAIRS directly.
   *
   * theta increments 0.005 rad per ms = 5 rad/s electrical.
   * Time for one full electrical revolution = 2*pi / 5 ≈ 1.26 s.
   * In 10 s you get ~7.95 electrical revolutions.
   * Mechanical revolutions in 10 s = 7.95 / pole_pairs.
   *   PP = 7  -> ~1.14 mech rev    (visible: clearly more than 1 turn)
   *   PP = 11 -> ~0.72 mech rev    (less than one turn)
   *   PP = 14 -> ~0.57 mech rev    (about half turn)
   * Set DEBUG_OPEN_LOOP_SECONDS to 0 once you trust the closed loop. */
  #define DEBUG_OPEN_LOOP_SECONDS 1
  if (DEBUG_OPEN_LOOP_SECONDS > 0) {
      float theta_dbg = 0.0f;
      const uint32_t t_end_dbg = HAL_GetTick()
                                 + (uint32_t)(DEBUG_OPEN_LOOP_SECONDS * 1000);
      while ((int32_t)(HAL_GetTick() - t_end_dbg) < 0) {
          FOC_OpenLoopUpdate(theta_dbg, 0.0f, 0.3f);
          theta_dbg += 0.005f;
          if (theta_dbg > 6.283185f) theta_dbg -= 6.283185f;
          HAL_Delay(1);
      }
      /* Re-align after the open-loop spin so theta_offset matches the new
       * resting position before closed loop takes over. */
      FOC_AlignRotor();
  }

  /* Start current-loop ISR, then enable position-loop on top.
   * pos_Kp  : torque per degree of error  (A/deg)
   * pos_Ki  : integral gain               (A/(deg·s))
   * iq_max  : maximum torque command (A), = pi_pos.out_max */
  g_foc.id_ref = 0.0f;
  g_foc.iq_ref = 0.0f;
  FOC_StartClosedLoopISR();
  FOC_EnablePositionMode(0.005f, 0.0005f, 0.0005f, 0.3f);

  /* Set initial target = current position (motor holds still). */
  /* Change g_foc.pos_ref_deg at run-time to command a new angle. */

  uint32_t next_plot = HAL_GetTick();
  uint32_t next_step = HAL_GetTick() + 3000;  /* first step after 3 s */

  /* Infinite loop — telemetry only; control runs in TIM1 update ISR. */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* VOFA+ FireWater protocol at ~100 Hz (every 10 ms).
     * 10 channels fit in 115200 baud at this rate. */
    if ((int32_t)(now - next_plot) >= 0) {
      next_plot = now + 10;

      /* VOFA+ channels (comma-separated, FireWater protocol):
       *  0: pos_ref   (deg, scaled /360 for display)
       *  1: pos_actual (deg, scaled /360)
       *  2: iq_ref    (A, from position PI)
       *  3: iq        (A, actual)
       *  4: Vq        (normalised) */
      printf("%.4f,%.4f,%.4f,%.4f,%.4f\n",
             g_foc.pos_ref_deg    / 360.0f,
             g_foc.theta_mech_deg / 360.0f,
             g_foc.iq_ref,
             g_foc.iq,
             g_foc.Vq);
    }

    /* Position step demo: toggle +90° every 3 seconds.
     * Replace this with your own setpoint source (UART, etc.). */
    if ((int32_t)(now - next_step) >= 0) {
      next_step = now + 3000;
      static float target = 0.0f;
      target += 90.0f;
      if (target >= 360.0f) target -= 360.0f;
      g_foc.pos_ref_deg = target;
    }
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


