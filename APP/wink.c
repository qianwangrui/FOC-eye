#include "wink.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "main.h"
#include "qwr_FOC.h"
#include "qwr_can_node.h"
#include "can_node_config.h"

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

  if (strncmp(cmd, "set ", 4) == 0) {
    g_foc.pos_ref_deg = strtof(cmd + 4, NULL);
    return;
  }

  printf("# unknown cmd: %s\n", cmd);
}
