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
    float left;
    float right;
} Motor_PWM;

typedef struct {
    float integral;
    float prev_error;
    float kp;
    float ki;
    float kd;
} PID_State;

extern PID_State pid_linear;
extern PID_State pid_angular;

#define DT 0.01f
#define TRACK_WIDTH 0.3f
#define MAX_SPEED 2.0f
#define PID_MAX_INTEGRAL 500.0f
#define WHEEL_RADIUS 0.02f

#define PID_OUT_MIN -1.0f
#define PID_OUT_MAX 1.0f

void DC_Motor_Init(DC_Motor *motor, TIM_HandleTypeDef *htim, uint32_t channel,
                   GPIO_TypeDef *dir1_port, uint16_t dir1_pin,
                   GPIO_TypeDef *dir2_port, uint16_t dir2_pin);

float DC_Motor_Set(DC_Motor *motor, float speed);

void PID_Init(PID_State *pid, float kp, float ki, float kd);

float PID_step(PID_State *pid, float expected, float measurement);

void compute_wheel_speeds(float linear_cmd, float angular_cmd,
                          float *speed_left, float *speed_right);

Motor_PWM tank_control(DC_Motor *left, DC_Motor *right,
                       float desired_angular, float desired_linear,
                       float real_angular, float real_linear);

#endif
