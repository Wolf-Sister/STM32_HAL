#include "PID.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


//增量式PID初始化
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_sum = 0.0f;
}
// 位置式PID计算
float PID_Compute(PID_Controller *pid, float measured_value) {
    float error, derivative, output;
    float p, i, d;
    // 计算误差
    arm_sub_f32(&(pid->setpoint), &measured_value, &error, 1); // error = setpoint - measured_value
    // 积分项累加
    arm_add_f32(&(pid->integral), &error, &(pid->integral), 1); // integral += error
    // 微分项
    arm_sub_f32(&error, &(pid->prev_error), &derivative, 1); // derivative = error - prev_error
    // 记录当前误差
    pid->prev_error = error;
    // 位置式PID公式
    arm_mult_f32(&(pid->Kp), &error, &p, 1);
    arm_mult_f32(&(pid->Ki), &(pid->integral), &i, 1);
    arm_mult_f32(&(pid->Kd), &derivative, &d, 1);
    arm_add_f32(&p, &i, &output, 1);
    arm_add_f32(&output, &d, &output, 1);
    return output;
}

