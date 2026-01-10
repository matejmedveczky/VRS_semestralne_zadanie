/*
 * control.h
 *
 *  Created on: Jan 10, 2026
 *      Author: matej
 */

#ifndef CONTROL_H
#define CONTROL_H

#include "stm32f3xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *dir1_port;
    uint16_t dir1_pin;
    GPIO_TypeDef *dir2_port;
    uint16_t dir2_pin;
} DC_Motor;

#define DT 0.01f
#define TRACK_WIDTH 0.3f
#define MAX_SPEED 2.0f
#define MAX_INTEGRAL 500.0f

void DC_Motor_Init(DC_Motor *motor, TIM_HandleTypeDef *htim, uint32_t channel,
                   GPIO_TypeDef *dir1_port, uint16_t dir1_pin,
                   GPIO_TypeDef *dir2_port, uint16_t dir2_pin);

void DC_Motor_Set(DC_Motor *motor, float speed);

void tank_control(DC_Motor *left, DC_Motor *right,
                  float v_linear_desired, float v_angular_desired,
                  float v_linear_actual, float v_angular_actual,
                  float KP_lin, float KI_lin, float KD_lin,
                  float KP_ang, float KI_ang, float KD_ang);
#endif
