#include "control.h"

static int zupt_counter = 0;
static bool can_integrate = true;


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

float DC_Motor_Set(DC_Motor *motor, float speed) {
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

    return (clamped / MAX_SPEED);
}

void PID_Init(PID_State *pid, float kp, float ki, float kd)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_output = 0.0f;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float PID_step(PID_State *pid, float expected, float measurement)
{
    float error = expected - measurement;


    if (error < 0.005 && error > -0.005){
    	return 0;
    }

    float derivative = (error - pid->prev_error) / DT;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if ((output < PID_OUT_MAX && output > PID_OUT_MIN) ||
        (output >= PID_OUT_MAX && error < 0) ||
        (output <= PID_OUT_MIN && error > 0))
    {
        pid->integral += error * DT;

        if (pid->integral > PID_MAX_INTEGRAL)
            pid->integral = PID_MAX_INTEGRAL;
        else if (pid->integral < -PID_MAX_INTEGRAL)
            pid->integral = -PID_MAX_INTEGRAL;
    }

    pid->prev_error = error;
    pid->prev_output = output;
    return output;
}

void compute_wheel_speeds(float linear_cmd,
                               float angular_cmd,
                               float *speed_left,
                               float *speed_right)
{
    *speed_left  = (linear_cmd - (angular_cmd * TRACK_WIDTH / 2.0f)) / WHEEL_RADIUS;
    *speed_right = (linear_cmd + (angular_cmd * TRACK_WIDTH / 2.0f)) / WHEEL_RADIUS;
}


Motor_PWM tank_control(DC_Motor *left, DC_Motor *right,
						PID_State *pid_angular, PID_State *pid_linear,
                       float desired_angular, float desired_linear,
                       float real_angular, float real_linear)
{
    bool should_be_stationary = (fabsf(desired_linear) < ZUPT_CMD_THRESHOLD);

    if (can_integrate == false){
    	pid_angular->ki = 0;
    	pid_linear->ki = 0;
    }

    if (should_be_stationary) {
        zupt_counter++;
        if (zupt_counter >= ZUPT_COUNT_THRESHOLD) {
            real_linear = 0.0f;
        }
    } else {
        zupt_counter = 0;

    }

    float linear_cmd  = PID_step(pid_linear,  desired_linear, real_linear);
    float angular_cmd = PID_step(pid_angular, desired_angular, real_angular);

    float speed_left, speed_right;
    compute_wheel_speeds(linear_cmd, angular_cmd, &speed_left, &speed_right);

    Motor_PWM pwm;
    pwm.left  = DC_Motor_Set(left,  speed_left);
    pwm.right = DC_Motor_Set(right, speed_right);


    if (pwm.left > (0.95*65536) || pwm.right > (0.95*65536))
    {
    	can_integrate = false;
    }
    else{
    	can_integrate = true;
    }

    return pwm;
}
