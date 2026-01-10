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

static PID_State pid_linear = {0};
static PID_State pid_angular = {0};

void tank_control(DC_Motor *left, DC_Motor *right,
                  float v_linear_desired, float v_angular_desired,
                  float v_linear_actual, float v_angular_actual,
                  float KP_lin, float KI_lin, float KD_lin,
                  float KP_ang, float KI_ang, float KD_ang) {

    // LINEAR
    float error_linear = v_linear_desired - v_linear_actual;
    pid_linear.integral += error_linear * DT;
    if (pid_linear.integral > MAX_INTEGRAL) pid_linear.integral = MAX_INTEGRAL;
    if (pid_linear.integral < -MAX_INTEGRAL) pid_linear.integral = -MAX_INTEGRAL;
    float derivative_linear = (error_linear - pid_linear.prev_error) / DT;
    float output_linear = KP_lin * error_linear + KI_lin * pid_linear.integral + KD_lin * derivative_linear;
    pid_linear.prev_error = error_linear;

    // ANGULAR
    float error_angular = v_angular_desired - v_angular_actual;
    pid_angular.integral += error_angular * DT;
    if (pid_angular.integral > MAX_INTEGRAL) pid_angular.integral = MAX_INTEGRAL;
    if (pid_angular.integral < -MAX_INTEGRAL) pid_angular.integral = -MAX_INTEGRAL;
    float derivative_angular = (error_angular - pid_angular.prev_error) / DT;
    float output_angular = KP_ang * error_angular + KI_ang * pid_angular.integral + KD_ang * derivative_angular;
    pid_angular.prev_error = error_angular;

    float speed_left = output_linear - (output_angular * TRACK_WIDTH / 2.0f);
    float speed_right = output_linear + (output_angular * TRACK_WIDTH / 2.0f);

    DC_Motor_Set(left, speed_left);
    DC_Motor_Set(right, speed_right);
}
