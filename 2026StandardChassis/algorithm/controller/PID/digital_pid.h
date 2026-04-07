/**
 * @file digital_pid.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __DIGITAL_PID_H__
#define __DIGITAL_PID_H__

#include <stdint.h>

#include "user_lib.h"

/*
 * 先暂时T:采样周期0.01s(10ms)
 * (根据输出大小调整合适单位)
 * 假设角速度/力矩：G(s)=1/(s+1)
 * 则前馈环节：Gf(s)=s+1
 *
 * ①输出：f_out=target'+(input-last_input)/T+input
 * input=(fliter(error))(t)即f_out=(input-last_input)/T+input
 * 从而f_out+pid_out=out,对时间积分(out)=output,输出output
 * &&
 * ②输出：f_out=kf*err_now
 */

// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式
typedef enum
{
    PID_IMPROVE_NONE_MOTION = 0b00000000,         // 0000 0000
    PID_INTEGRAL_LIMIT = 0b00000001,              // 0000 0001
    PID_CHANGING_INTEGRATION_RATE = 0b00000010,   // 0000 0010
    PID_TRAPEZOID_INTEGRAL = 0b00000100,          // 0000 0100
    PID_PROPORTIONAL_ON_MEASUREMENT = 0b00001000, // 0000 1000
    PID_OUTPUT_FILTER = 0b00010000,               // 0001 0000
    PID_DERIVATIVE_ON_MEASUREMENT = 0b00100000,   // 0010 0000
    PID_DERIVATIVE_FILTER = 0b01000000,           // 0100 0000
    PID_ERROR_HANDLE = 0b10000000,                // 1000 0000 检测错误
    PID_RAMP_TARGET = 0b100000000,
} PID_improvement_e;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Kf; // 前馈系数

    float i_term;
    float p_out;
    float i_out;
    float d_out;
    float f_out;

    float target;
    float measure;
    float last_target;
    float last_measure;
    float dead_band;

    float dt; // 即使不使用，也能监测运行频率

    float output_LPF_RC; // 输出滤波器 RC = 1/omegac

    float ref_p; // 设定值
    float fab_p; // 反馈值

    float ci_coefA; //>=最大误差	// 变速积分 For Changing Integral
    float ci_coefB; //<=最小误差	// 变速积分 ITerm = Err*((A-user_abs(err)+B)/A)  when B<|err|<A+B
    float d_out_last;
    float derivative_LPF_RC; // 微分滤波器系数
    float derivative_gama;   // 微分先行系数

    float err_pre;
    float err_last;
    float err_now;

    float output_max;
    float integral_limit; // 积分限幅
    float forward_limit;  // 前馈限幅

    float output;
    float output_last;

    uint32_t DWT_cnt;

    PID_improvement_e improve; // PID开启优化选项
    uint8_t lost_cnt;
    uint16_t error_cnt;
    uint8_t error_flag;
    uint16_t lost_integral_cnt; // 丢失积分
    ramp_function_source_t *ramp_target;
} __attribute__((__packed__)) digital_PID_t;

typedef enum
{
    ANGLE = 0,     // 速度环，使用增量式
    SPEED = 1,     // 角度环，使用位置式
    DUAL_LOOP = 2, // 双环控制，外环角度内环速度
} PID_e;

extern float Digital_PID_Position(digital_PID_t *hpid, float val_now, float target_now);

extern float Digital_PID_Increment(digital_PID_t *hpid, float val_now, float target_now);

extern void Digital_PID_Clear(digital_PID_t *hpid);

#endif
