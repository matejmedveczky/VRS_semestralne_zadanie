/*
 * control.c
 *
 *  Created on: Jan 10, 2026
 *      Author: matej
 */

#include "control.h"

typedef struct {
    float integral;
    float prev_error;
} PID_State;

static PID_State pid_left = {0};
static PID_State pid_right = {0};

void DC_Motor_Init(DC_Motor *motor, TIM_HandleTypeDef *htim, uint32_t channel,
                   GPIO_TypeDef *dir1_port, uint16_t dir1_pin,
                   GPIO_TypeDef *dir2_port, uint16_t dir2_pin) {
    motor->htim = htim;
    motor->channel = channel;
    motor->dir1_port = dir1_port;
    motor->dir1_pin = dir1_pin;
    motor->dir2_port = dir2_port;
    motor->dir2_pin = dir2_pin;
}

void DC_Motor_Set(DC_Motor *motor, float speed) {
    uint16_t max_pwm = __HAL_TIM_GET_AUTORELOAD(motor->htim);

    float clamped = speed;
    if (clamped > MAX_SPEED) clamped = MAX_SPEED;
    if (clamped < -MAX_SPEED) clamped = -MAX_SPEED;

    if (clamped >= 0) {
        HAL_GPIO_WritePin(motor->dir1_port, motor->dir1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->dir2_port, motor->dir2_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(motor->dir1_port, motor->dir1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->dir2_port, motor->dir2_pin, GPIO_PIN_RESET);
        clamped = -clamped;
    }

    uint16_t pwm = (uint16_t)((clamped / MAX_SPEED) * max_pwm);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, pwm);
}

void tank_control(DC_Motor *left, DC_Motor *right,
                  float v_linear_desired, float v_angular_desired,
                  float v_linear_actual, float v_angular_actual, float KP, float KI, float KD) {

    float desired_left = v_linear_desired - (v_angular_desired * TRACK_WIDTH / 2.0f);
    float desired_right = v_linear_desired + (v_angular_desired * TRACK_WIDTH / 2.0f);

    float actual_left = v_linear_actual - (v_angular_actual * TRACK_WIDTH / 2.0f);
    float actual_right = v_linear_actual + (v_angular_actual * TRACK_WIDTH / 2.0f);

    float error_left = desired_left - actual_left;
    pid_left.integral += error_left * DT;
    if (pid_left.integral > MAX_INTEGRAL) pid_left.integral = MAX_INTEGRAL;
    if (pid_left.integral < -MAX_INTEGRAL) pid_left.integral = -MAX_INTEGRAL;
    float derivative_left = (error_left - pid_left.prev_error) / DT;
    float output_left = KP * error_left + KI * pid_left.integral + KD * derivative_left;
    pid_left.prev_error = error_left;

    float error_right = desired_right - actual_right;
    pid_right.integral += error_right * DT;
    if (pid_right.integral > MAX_INTEGRAL) pid_right.integral = MAX_INTEGRAL;
    if (pid_right.integral < -MAX_INTEGRAL) pid_right.integral = -MAX_INTEGRAL;
    float derivative_right = (error_right - pid_right.prev_error) / DT;
    float output_right = KP * error_right + KI * pid_right.integral + KD * derivative_right;
    pid_right.prev_error = error_right;

    DC_Motor_Set(left, output_left);
    DC_Motor_Set(right, output_right);
}

