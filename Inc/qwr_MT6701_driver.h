#ifndef _MT6701_DRIVER_H_
#define _MT6701_DRIVER_H_

#include "stm32g4xx_hal.h"

void MT6701_GPIO_Init(void);
void MT6701_SPI_Init(void);
uint16_t MT6701_ReadAngle_SSI(void);
float MT6701_GetAngleDeg(void);

#endif // _MT6701_DRIVER_H_