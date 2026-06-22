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

static const char *cmd_skip_ws(const char *s)
{
  while (s != NULL && (*s == ' ' || *s == '\t')) {
    s++;
  }
  return s;
}

static void request_angle_or_warn(float deg)
{
  if (!g_foc.aligned) {
    printf("# error: not calibrated — run 'cal align' then 'cal save'\r\n");
    return;
  }
  g_foc.pos_ref_deg = deg;
  FOC_RequestAngle(deg);
}

void APP_FOC_StartMotor(void)
{
  if (!g_foc.aligned) {
    return;
  }
  FOC_InitPositionMode(MOTOR_POS_KP, MOTOR_POS_KI, MOTOR_POS_KD, MOTOR_IQ_MAX);
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
  request_angle_or_warn(end_deg);
  HAL_Delay(35);

  g_foc.pos_ref_deg = start_deg;
  request_angle_or_warn(start_deg);
  HAL_Delay(350);
}

void APP_RunCommand(const char *cmd)
{
  if (cmd == NULL) {
    return;
  }

  cmd = cmd_skip_ws(cmd);
  if (*cmd == '\0') {
    return;
  }

  if ((strcmp(cmd, "blink") == 0) || (strcmp(cmd, "blink1") == 0)) {
    APP_LowerEyelidBlink();
    printf("# blink done\n");
    return;
  }

  if (strncmp(cmd, "pos", 3) == 0 &&
      (cmd[3] == '\0' || cmd[3] == ' ' || cmd[3] == '\t')) {
    const char *arg = cmd_skip_ws(cmd + 3);
    float deg = g_foc.pos_ref_deg + strtof(arg, NULL);
    request_angle_or_warn(deg);
    return;
  }

  if (strncmp(cmd, "all", 3) == 0 &&
      (cmd[3] == '\0' || cmd[3] == ' ' || cmd[3] == '\t')) {
    const char *p = cmd_skip_ws(cmd + 3);
    int8_t angles[CAN_MOTOR_COUNT];
    char *end = NULL;

    for (uint8_t i = 0U; i < CAN_MOTOR_COUNT; i++) {
      p = cmd_skip_ws(p);
      if (*p == '\0') {
        printf("# all: need %u angles, e.g. all -30 -50 -30 -50 -50 -50\r\n",
               (unsigned)CAN_MOTOR_COUNT);
        return;
      }
      float deg = strtof(p, &end);
      if (end == p) {
        printf("# all: bad angle near '%s'\r\n", p);
        return;
      }
      angles[i] = clamp_angle_i8(deg);
      p = end;
    }
    static uint8_t seq = 0U;
    CAN_ForwardAngles(angles, seq);
    printf("# all ok seq=%u\r\n", (unsigned)seq);
    seq++;
    return;
  }

  if (strncmp(cmd, "cal", 3) == 0 &&
      (cmd[3] == '\0' || cmd[3] == ' ' || cmd[3] == '\t')) {
    const char *sub = cmd_skip_ws(cmd + 3);
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

  if (strncmp(cmd, "set", 3) == 0 &&
      (cmd[3] == '\0' || cmd[3] == ' ' || cmd[3] == '\t')) {
    const char *arg = cmd_skip_ws(cmd + 3);
    float deg = strtof(arg, NULL);
    request_angle_or_warn(deg);
    return;
  }

  if (strcmp(cmd, "enc") == 0) {
    float enc = MT6701_GetAngleDeg();
    printf("# enc_raw=%.2f enc=%.2f target=%.2f err=%.2f vel=%.1f\r\n",
           (double)enc,
           (double)FOC_EncoderSignedDeg(enc),
           (double)g_foc.pos_ref_deg,
           (double)FOC_GetPosErrDeg(),
           (double)g_foc.vel_deg_s);
    return;
  }

  printf("# unknown cmd: %s\r\n", cmd);
}
