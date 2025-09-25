#include "PID.h"

static float kp = 1.0f; // Proportional gain
static float ki = 0.0f; // Integral gain
static float kd = 0.0f; // Derivative gain

static float previous_error = 0.0f;
static float integral = 0.0f;

void PID_Init(PID_Controller *pid, float proportional, float integral_gain, float derivative) {
    pid->kp = proportional;
    pid->ki = integral_gain;
    pid->kd = derivative;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
}

float PID_Compute(PID_Controller *pid, float setpoint, float measured_value) {
    float error = setpoint - measured_value;
    pid->integral += error;
    float derivative = error - pid->previous_error;

    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    pid->previous_error = error;

    return output;
}