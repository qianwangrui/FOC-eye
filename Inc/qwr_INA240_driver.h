#ifndef _INA240_DRIVER_H_
#define _INA240_DRIVER_H_

#include "stm32g4xx_hal.h"

void INA240_GPIO_Init(void);
void INA240_ADC_Init(void);
float INA240_ReadCurrent_IU(void);
float INA240_ReadCurrent_IV(void);

#endif // _INA240_DRIVER_H_
