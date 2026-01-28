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

typedef struct {
    float pwm_left;
    float pwm_right;
    float u_ang;
    float e_ang;
    float u_lin;
    float e_lin;
} Reg_State;

#define DT 0.01f
#define TRACK_WIDTH 0.22f
#define MAX_SPEED 2.0f
#define MAX_INTEGRAL 2.0f
#define WHEEL_RADIUS 0.02f

void DC_Motor_Init(DC_Motor *motor, TIM_HandleTypeDef *htim, uint32_t channel,
                   GPIO_TypeDef *dir1_port, uint16_t dir1_pin,
                   GPIO_TypeDef *dir2_port, uint16_t dir2_pin);

float DC_Motor_Set(DC_Motor *motor, float speed);

Reg_State tank_control(DC_Motor *left, DC_Motor *right,
                  float desired_angular, float desired_linear,
                  float real_angular, float real_linear,
                  float KP_lin, float KI_lin, float KD_lin,
                  float KP_ang, float KI_ang, float KD_ang);
#endif
