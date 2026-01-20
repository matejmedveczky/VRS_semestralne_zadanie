#include "control.h"

typedef struct {
    float integral;
    float prev_error;
} PID_State;

static PID_State pid_linear = {0};
static PID_State pid_angular = {0};

void DC_Motor_Init(DC_Motor *motor, TIM_HandleTypeDef *htim, uint32_t channel,
                   GPIO_TypeDef *dir1_port, uint16_t dir1_pin,
                   GPIO_TypeDef *dir2_port, uint16_t dir2_pin) {
    motor->htim = htim;
    motor->channel = channel;
    motor->dir1_port = dir1_port;
    motor->dir1_pin = dir1_pin;
    motor->dir2_port = dir2_port;
    motor->dir2_pin = dir2_pin;

    HAL_TIM_PWM_Start(htim, channel);
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
				  float desired_linear, float desired_angular,
				  float real_angular, float real_linear,
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
