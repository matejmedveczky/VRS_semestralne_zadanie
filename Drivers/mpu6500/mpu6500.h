#ifndef MPU6500_H
#define MPU6500_H

#include "stm32f3xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

bool MPU6500_Init(I2C_HandleTypeDef *hi2c);

bool MPU6500_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz);

/* Helpers used by main.c */
float read_gyroscope_z(void);       /* rad/s */
float read_accelerometer_x(void);   /* m/s^2 */

#endif
