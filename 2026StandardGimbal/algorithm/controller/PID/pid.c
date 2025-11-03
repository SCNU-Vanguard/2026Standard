/**
******************************************************************************
* @file    bsp_can.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/
#include "pid.h"

/**
 * @brief 初始化PID,设置参数和启用的优化环节,将其他数据置零
 * @param config PID初始化设置
 * @return PID_t* PID实例指针
 */
PID_t *PID_Init(PID_t *config)
{
    if (config == NULL)
    {
        return NULL;
    }

    PID_t *pid = (PID_t *)malloc(sizeof(PID_t));
    memset(pid, 0, sizeof(PID_t));

    pid->kp = config->kp;
    pid->ki = config->ki;
    pid->kd = config->kd;
    pid->integral_limit = config->integral_limit;
    pid->output_limit = config->output_limit;
    pid->dead_band = config->dead_band;

    return pid;
}

float Value_Limit(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }
    else
    {
        return value;
    }
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PID_Position(PID_t *pid, float measure, float target)
{
    // 保存上次的测量值和误差,计算当前error
    pid->measure = measure;
    pid->target = target;

    while (pid->target > PI)
    {
        pid->target = pid->target - 2 * PI;
    }
    while (pid->target < -PI)
    {
        pid->target = pid->target + 2 * PI;
    }

    pid->error = pid->target - pid->measure;

    while (pid->error > PI)
    {
        pid->error = pid->error - 2 * PI;
    }
    while (pid->error < -PI)
    {
        pid->error = pid->error + 2 * PI;
    }

    // 如果在死区外,则计算PID
    if (pid_abs(pid->error) > pid->dead_band)
    {
        // 基本的pid计算,使用位置式
        pid->p_out = pid->kp * pid->error;
        pid->i_term = pid->ki * pid->error;
        pid->i_out += pid->i_term;
        pid->d_out = pid->kd * (pid->error - pid->last_error);

        pid->i_out = Value_Limit(pid->i_out, -pid->integral_limit, pid->integral_limit); // 积分限幅

        pid->output = pid->p_out + pid->i_out + pid->d_out; // 计算输出

        pid->output = Value_Limit(pid->output, -pid->output_limit, pid->output_limit); // 输出限幅
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->output = 0;
        pid->i_term = 0;
    }

    // 保存当前数据,用于下次计算
    pid->last_target = pid->target;
    pid->last_measure = pid->measure;
    pid->last_output = pid->output;
    pid->last_d_out = pid->d_out;
    pid->last_error = pid->error;

    return pid->output;
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PID_Increment(PID_t *pid, float measure, float target)
{
    // 保存上次的测量值和误差,计算当前error
    pid->measure = measure;
    pid->target = target;
    pid->error = pid->target - pid->measure;

    // 如果在死区外,则计算PID
    if (pid_abs(pid->error) > pid->dead_band)
    {
        // 基本的pid计算,使用增量式
        pid->p_out = pid->kp * (pid->error - pid->last_error);
        pid->i_term = pid->ki * pid->error;
        pid->i_out = pid->i_term;
        pid->d_out = pid->kd * (pid->error - 2.0f * pid->last_error + pid->pre_error);

        pid->i_out = Value_Limit(pid->i_out, -pid->integral_limit, pid->integral_limit); // 积分限幅

        pid->output += (pid->p_out + pid->i_out + pid->d_out); // 计算输出

        pid->output = Value_Limit(pid->output, -pid->output_limit, pid->output_limit); // 输出限幅
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->output = 0;
        pid->i_term = 0;
    }

    // 保存当前数据,用于下次计算
    pid->last_target = pid->target;
    pid->last_measure = pid->measure;
    pid->last_output = pid->output;
    pid->last_d_out = pid->d_out;
    pid->pre_error = pid->last_error;
    pid->last_error = pid->error;

    return pid->output;
}
