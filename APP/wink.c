#include "wink.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "main.h"
#include "qwr_FOC.h"
#include "qwr_can_node.h"
#include "can_node_config.h"
#include "nvm_cal.h"
#include "motor_config.h"

static int8_t clamp_angle_i8(float deg)
{
  if (deg > 127.0f) {
    deg = 127.0f;
  }
  if (deg < -128.0f) {
    deg = -128.0f;
  }
  if (deg >= 0.0f) {
    return (int8_t)(deg + 0.5f);
  }
  return (int8_t)(deg - 0.5f);
}

void APP_FOC_StartMotor(void)
{
  if (FOC_IsRunning()) {
    return;
  }
  g_foc.id_ref = 0.0f;
  g_foc.iq_ref = 1.0f;
  FOC_StartClosedLoopISR();
  FOC_EnablePositionMode(MOTOR_POS_KP, MOTOR_POS_KI, MOTOR_POS_KD, MOTOR_IQ_MAX);
  g_foc.pos_ref_deg = -15.0f;
}

static void cal_cmd_align(void)
{
  if (FOC_IsRunning()) {
    FOC_StopClosedLoopISR();
  }
  FOC_AlignRotor();
  printf("# cal align ok theta=%.6f rad\r\n", (double)g_foc.theta_offset);
  APP_FOC_StartMotor();
}

static void cal_cmd_save(void)
{
  if (!g_foc.aligned) {
    printf("# cal: run 'cal align' first\r\n");
    return;
  }
  int rc = NVM_Cal_Save(g_foc.theta_offset);
  if (rc == 0) {
    printf("# cal save ok\r\n");
  } else {
    printf("# cal save fail rc=%d\r\n", rc);
  }
}

static void cal_cmd_show(void)
{
  NVM_Cal_Show(g_foc.theta_offset);
}

void APP_LowerEyelidBlink(void)
{
  const float start_deg = -15.0f;
  const float end_deg   = -52.0f;

  // g_foc.pos_ref_deg = start_deg;
  // HAL_Delay(30000);

  g_foc.pos_ref_deg = end_deg;
  HAL_Delay(35);

  g_foc.pos_ref_deg = start_deg;
  HAL_Delay(350);
}

void APP_RunCommand(const char *cmd)
{
  if (cmd == NULL) {
    return;
  }

  if ((strcmp(cmd, "blink") == 0) || (strcmp(cmd, "blink1") == 0)) {
    APP_LowerEyelidBlink();
    printf("# blink done\n");
    return;
  }

  if (strncmp(cmd, "pos ", 4) == 0) {
    g_foc.pos_ref_deg += strtof(cmd + 4, NULL);
    return;
  }

  if (strncmp(cmd, "all ", 4) == 0) {
#if CAN_NODE_IS_GATEWAY
    const char *p = cmd + 4;
    int8_t angles[CAN_MOTOR_COUNT];
    char *end = NULL;

    for (uint8_t i = 0U; i < CAN_MOTOR_COUNT; i++) {
      while (*p == ' ') {
        p++;
      }
      float deg = strtof(p, &end);
      if (end == p) {
        printf("# all: need %u angles\n", (unsigned)CAN_MOTOR_COUNT);
        return;
      }
      angles[i] = clamp_angle_i8(deg);
      p = end;
    }
    static uint8_t seq = 0U;
    CAN_GatewayForwardAngles(angles, seq);
    printf("# all ok seq=%u\n", (unsigned)seq);
    seq++;
#else
    printf("# all: gateway only\n");
#endif
    return;
  }

  if (strncmp(cmd, "cal ", 4) == 0) {
    const char *sub = cmd + 4;
    if (strcmp(sub, "align") == 0) {
      cal_cmd_align();
    } else if (strcmp(sub, "save") == 0) {
      cal_cmd_save();
    } else if (strcmp(sub, "show") == 0) {
      cal_cmd_show();
    } else {
      printf("# cal: use align | save | show\r\n");
    }
    return;
  }

  if (strncmp(cmd, "set ", 4) == 0) {
    g_foc.pos_ref_deg = strtof(cmd + 4, NULL);
    return;
  }

  printf("# unknown cmd: %s\n", cmd);
}
