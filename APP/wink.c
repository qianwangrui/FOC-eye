#include "wink.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "qwr_FOC.h"

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

  if (strncmp(cmd, "set ", 4) == 0) {
    g_foc.pos_ref_deg = strtof(cmd + 4, NULL);
    return;
  }

  printf("# unknown cmd: %s\n", cmd);
}
