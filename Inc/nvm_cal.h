#ifndef NVM_CAL_H
#define NVM_CAL_H

#include <stdint.h>

/* Last 2 KB page of 128 KB flash (STM32G431). Not included in firmware .bin. */
#define NVM_CAL_FLASH_ADDR   0x0801F800U
#define NVM_CAL_FLASH_PAGE   63U

int  NVM_Cal_TryLoad(float *theta_offset_rad);
int  NVM_Cal_Save(float theta_offset_rad);
void NVM_Cal_Show(float runtime_theta_rad);

#endif /* NVM_CAL_H */
