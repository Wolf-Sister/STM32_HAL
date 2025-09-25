#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float Kp;      // Proportional gain
    float Ki;      // Integral gain
    float Kd;      // Derivative gain
    float setpoint; // Desired value
    float integral; // Integral value
    float previous_error; // Previous error value
} PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd);
float PID_Compute(PID_Controller *pid, float measured_value);

#endif // PID_H