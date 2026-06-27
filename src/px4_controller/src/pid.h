#ifndef __PID_H
#define __PID_H

#include "main.h"

// 初始化 PID 控制器
inline void PID_Init(PIDController* pid, float p, float i, float d)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->ans_error = 0.0f;
    pid->previous_error = 0.0f;
}

// PID 计算函数（位置式）
inline float PID_Calculate(PIDController* pid, float target, float current)
{
    float error = target - current;

    // 积分项
    pid->ans_error += error;

    // 微分项
    float slope = (error - pid->previous_error) / 0.033333f;
    pid->previous_error = error;

    // PID 输出计算
    float output = pid->kp * error + pid->ki * pid->ans_error - pid->kd * slope;
    return output;
}

#endif // __PID_H