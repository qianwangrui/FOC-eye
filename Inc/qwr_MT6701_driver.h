#ifndef _MT6701_DRIVER_H_
#define _MT6701_DRIVER_H_

#include "stm32g4xx_hal.h"

/* Magnetic field status (4-bit field Mg[3:0]):
 *   Mg[3]    : 1 = Loss of Track          (tracking lost)
 *   Mg[2]    : 1 = Push Button Detected   (axial press of magnet)
 *   Mg[1:0]  : 2-bit enum, see MT6701_MAG_* below
 */
#define MT6701_STATUS_LOSS_TRACK  (1u << 3)  /* Mg[3] */
#define MT6701_STATUS_PUSH_BTN    (1u << 2)  /* Mg[2] */
#define MT6701_STATUS_MAG_MASK    (0x3u)     /* Mg[1:0] extraction mask */

/* Values of Mg[1:0] (use: (status & MT6701_STATUS_MAG_MASK) == MT6701_MAG_*) */
#define MT6701_MAG_NORMAL     0   /* field OK */
#define MT6701_MAG_TOO_STRONG 1   /* magnet too close */
#define MT6701_MAG_TOO_WEAK   2   /* magnet too far / missing */
#define MT6701_MAG_RESERVED   3   /* reserved */

void MT6701_SPI_Init(void);
uint16_t MT6701_ReadAngle_SSI(void);
float MT6701_GetAngleDeg(void);

/* Read full 24-bit frame. Any output pointer may be NULL if not needed. */
void     MT6701_ReadFrame(uint16_t *angle14, uint8_t *status4, uint8_t *crc6);
uint8_t  MT6701_GetStatus(void);

#endif // _MT6701_DRIVER_H_