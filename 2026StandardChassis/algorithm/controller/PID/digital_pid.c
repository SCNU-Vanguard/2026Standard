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

#include "digital_pid.h"

#include "bsp_dwt.h"

/* ----------------------------下面是pid优化环节的实现---------------------------- */
// TODO(GUATAI)dt获取需要进一步联动DWT内核定时器
//  变速积分(误差小时积分作用更强)
// 完整变速积分 基于积分分离优化的思想，根据系统偏差大小改变积分的速度，偏差越大，积分越慢，反之则越快，
// 形成连续的变化过程。为此，设置变速积分系数alpha
// 当error增大时，f减小，反之增大，其值在0~1变化。
// static float err_absmax,err_absmin,alpha = 0.0;
//
//	if (user_abs(error) <= err_absmin)
//	{
//		alpha = 1.0;
//	}
//	else if (user_abs(error) > err_absmax)
//	{
//		alpha = 0.0;
//	}
//	else
//	{
//		alpha = (err_absmax - user_abs(error)) / (err_absmax - err_absmin);
//	}
//	pid->i_term=pid->i_term*alpha
// err_absmax=A+B,err_absmin=B 人为设定两个值即可
//  微分先行(仅使用反馈值而不计参考输入的微分)
// 完整微分先行static float d1, d2, d3, temp,gama微分先行系数;
//				temp = pid->gama * pid->kd + pid->kp;
//				d3 = pid->kd / temp;
//				d2 = (pid->kd + pid->kp) / temp;
//				d1 = pid->gama * d3;
//				pid->delta_current_value = pid->current_value
//						- pid->last_current_value;
//				pid->d_out = d1 * pid->d_out_last
//						+ d2 * pid->delta_current_value
//						+ d3 * pid->last_delta_current_value;

static void F_Output_Limit(digital_PID_t *hpid);
static void F_Changing_Integral_Rate(digital_PID_t *hpid);
static void F_Trapezoid_Intergral(digital_PID_t *hpid);
static void F_Integral_Limit(digital_PID_t *hpid);
static void F_Derivative_Filter(digital_PID_t *hpid);
static void F_Derivative_On_Measurement(digital_PID_t *hpid);
static void F_Feedforward_Out(digital_PID_t *hpid);
static void F_Smooth_Target(digital_PID_t *hpid);
static void F_Output_Filter(digital_PID_t *hpid);
static void F_Error_Handler(digital_PID_t *hpid);

/*
 * @brief  	位置式pid控制器
 * @param
 * 		hpid			pid结构体
 * 		val_now			获取到的当前值
 * 		target_now		目标值
 * @retval 	控制量
 *
 */
float Digital_PID_Position(digital_PID_t *hpid, float val_now, float target_now)
{
    hpid->target = target_now;
    hpid->measure = val_now;

    hpid->err_now = hpid->target - hpid->measure;

    hpid->dt = DWT_GetDeltaT(&hpid->DWT_cnt);

    if (hpid->improve & PID_ERROR_HANDLE)
    {
        F_Error_Handler(hpid);
    }

    if (user_abs(hpid->err_now) > hpid->dead_band)
    {
        hpid->p_out = hpid->Kp * hpid->err_now;
        hpid->i_term = hpid->Ki * hpid->err_now;
        hpid->d_out = hpid->Kd * (hpid->err_now - hpid->err_last);

        if (hpid->improve & PID_TRAPEZOID_INTEGRAL)
        {
            F_Trapezoid_Intergral(hpid);
        }

        if (hpid->improve & PID_CHANGING_INTEGRATION_RATE)
        {
            F_Changing_Integral_Rate(hpid);
        }

        if (hpid->improve & PID_INTEGRAL_LIMIT)
        {
            F_Integral_Limit(hpid);
        }

		if (hpid->improve & PID_DERIVATIVE_ON_MEASUREMENT)
		{
			F_Derivative_On_Measurement(hpid); // 别用
		}

        if (hpid->improve & PID_DERIVATIVE_FILTER)
        {
            F_Derivative_Filter(hpid);
        }
        hpid->i_out += hpid->i_term;

        hpid->output = hpid->p_out + hpid->i_out + hpid->d_out;
        hpid->lost_integral_cnt = 0;
    }
    else
    {
        hpid->output = 0;
        hpid->i_term = 0;
        if(hpid->lost_integral_cnt < 1000)
        {
            hpid->lost_integral_cnt++;
        }
        else
        {
            hpid->i_out = 0;
        }
    }

    F_Output_Limit(hpid);

    hpid->err_pre = hpid->err_last;
    hpid->err_last = hpid->err_now;

    hpid->output_last = hpid->output;

    hpid->last_measure = hpid->measure;
    hpid->last_target = hpid->target;

    return hpid->output;
}

/*
 * @brief  	增量式pid控制器
 * @param
 * 		hpid			pid结构体
 * 		val_now			当前值
 * 		target_now		目标值
 * @retval 	控制量
 *
 */
float Digital_PID_Increment(digital_PID_t *hpid, float val_now, float target_now)
{
    if (hpid->target != target_now)
    {
        hpid->last_target = hpid->target;
        hpid->target = target_now;
    }

    hpid->last_measure = hpid->measure;
    hpid->measure = val_now;

    if (hpid->improve & PID_RAMP_TARGET)
    {
        F_Smooth_Target(hpid);
    }
    else
    {
        hpid->ref_p = hpid->target;
    }

    if (hpid->improve & PID_ERROR_HANDLE)
    {
        F_Error_Handler(hpid);
        if (hpid->target == 0)
        {
            if (hpid->lost_cnt != 225)
            {
                hpid->lost_cnt++;
            }
            else
            {
                hpid->lost_cnt = 225;
            }
        }
    }

    hpid->fab_p = hpid->measure;
    hpid->err_now = hpid->ref_p - hpid->fab_p;

    hpid->dt = DWT_GetDeltaT(&hpid->DWT_cnt);

    if (user_abs(hpid->err_now) > hpid->dead_band)
    {
        hpid->p_out = hpid->Kp * (hpid->err_now - hpid->err_last);
        hpid->i_out = hpid->Ki * (hpid->err_now);

		if(hpid->improve & PID_DERIVATIVE_ON_MEASUREMENT)
		{
			F_Derivative_On_Measurement(hpid);
		}
		else
		{
			hpid->d_out = hpid->Kd * ( hpid->err_now - 2 * hpid->err_last + hpid->err_pre );
		}

		if(hpid->improve & PID_DERIVATIVE_FILTER)
		{
			F_Derivative_Filter(hpid);
		}

        hpid->output += hpid->p_out + hpid->i_out + hpid->d_out;

        if (hpid->improve & PID_PROPORTIONAL_ON_MEASUREMENT)
        {
            F_Feedforward_Out(hpid);
        }

        hpid->output += hpid->f_out;

        hpid->lost_cnt = 0;
    }
    else
    {
        hpid->output = hpid->output_last;
    }

    if (hpid->improve & PID_OUTPUT_FILTER)
    {
        F_Output_Filter(hpid);
    }

    F_Output_Limit(hpid);

    if (hpid->lost_cnt > 254)
    {
        hpid->output = 0.0f;
    }

    hpid->err_pre = hpid->err_last;
    hpid->err_last = hpid->err_now;

    hpid->output_last = hpid->output;

    return hpid->output;
}

/*
 * @brief  	清除PID积分值和输出
 * @param	pid结构体
 * @retval 	无
 *
 */
void Digital_PID_Clear(digital_PID_t *hpid)
{
    hpid->err_pre = 0.0f;
    hpid->err_last = 0.0f;
    hpid->err_now = 0.0f;
    hpid->target = 0.0f;
    hpid->measure = 0.0f;
    hpid->last_target = 0.0f;
    hpid->last_measure = 0.0f;
    hpid->p_out = 0.0f;
    hpid->i_out = 0.0f;
    hpid->d_out = 0.0f;
    hpid->d_out_last = 0.0f;
    hpid->i_term = 0.0f;
    hpid->output = 0.0f;
    hpid->output_last = 0.0f;
}

// 输出限幅
static void F_Output_Limit(digital_PID_t *hpid)
{
    if (hpid->output > hpid->output_max)
    {
        hpid->output = hpid->output_max;
    }
    if (hpid->output < -(hpid->output_max))
    {
        hpid->output = -(hpid->output_max);
    }
}

// 梯形积分
static void F_Trapezoid_Intergral(digital_PID_t *hpid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    hpid->i_term = hpid->Ki * ((hpid->err_now + hpid->err_last) / 2);
}

// 变速积分
static void F_Changing_Integral_Rate(digital_PID_t *hpid)
{
    if (hpid->err_now * hpid->i_out > 0)
    {
        // 积分呈累积趋势(误差小时直接返回即Full integral)
        if (user_abs(hpid->err_now) <= hpid->ci_coefB)
        {
            return;
        }
        // 本次误差处于一定范围内，积分变速即*（0~1）的值
        if (user_abs(hpid->err_now) <= (hpid->ci_coefA + hpid->ci_coefB))
        {
            hpid->i_term *= (hpid->ci_coefA - user_abs(hpid->err_now) + hpid->ci_coefB) / hpid->ci_coefA;
        }
        // 误差大时直接清零本次积分
        else
        { // 最大阈值,不使用积分
            hpid->i_term = 0;
        }
    }
}

// 积分限幅
static void F_Integral_Limit(digital_PID_t *hpid)
{
    static float temp_output, temp_iout;

    temp_iout = hpid->i_out + hpid->i_term;
    temp_output = hpid->p_out + hpid->i_out + hpid->d_out;

    // 输出已经大于输出上下限限幅，清零本次积分
    if (user_abs(temp_output) > hpid->output_max)
    {
        if (temp_output > 0 && (hpid->err_now * hpid->i_out > 0)) // 积分却还在累积
        {
            hpid->i_term = 0; // 当前积分项置零
        }
        if (temp_output < 0 && (hpid->err_now * hpid->i_out < 0)) // 积分却还在累积
        {
            hpid->i_term = 0; // 当前积分项置零
        }
    }

    // 积分输出超过上限，等于最大积分输出
    if (temp_iout > hpid->integral_limit)
    {
        hpid->i_term = 0;
        hpid->i_out = hpid->integral_limit;
    }
    // 积分输出超过下限，等于最小积分输出
    if (temp_iout < -hpid->integral_limit)
    {
        hpid->i_term = 0;
        hpid->i_out = -hpid->integral_limit;
    }
}

// 只能一个用，测试用，无实际作用
static void F_Derivative_On_Measurement(digital_PID_t *hpid)
{
	hpid->d_out = hpid->Kd * (hpid->last_measure - hpid->measure);
//    float d1, d2, d3, temp;
//    static float delta_measure_error, last_delta_measure_error = 0.0f;
//    temp = hpid->derivative_gama * hpid->Kd + hpid->Kp;
//    d3 = hpid->Kd / temp;
//    d2 = (hpid->Kd + hpid->Kp) / temp;
//    d1 = hpid->derivative_gama * d3;
//
//    last_delta_measure_error = delta_measure_error;
//    delta_measure_error = hpid->measure - hpid->last_measure;
//
//    hpid->d_out_last = hpid->d_out;
//    hpid->d_out = d1 * hpid->d_out_last + d2 * delta_measure_error + d3 * last_delta_measure_error;
}

// 微分滤波(采集微分时,滤除高频噪声)
// 不完全微分
static void F_Derivative_Filter(digital_PID_t *hpid)
{
    hpid->d_out = hpid->d_out * (1 - hpid->derivative_LPF_RC) + hpid->d_out_last * hpid->derivative_LPF_RC;
}

// 前馈输出
static void F_Feedforward_Out(digital_PID_t *hpid)
{
    hpid->f_out = hpid->Kf * (hpid->target - hpid->last_target);
    hpid->f_out = float_constrain(hpid->f_out, -hpid->forward_limit, hpid->forward_limit);
    if (hpid->p_out > hpid->output_max)
    {
        hpid->p_out = hpid->output_max;
    }
    if (hpid->p_out < -(hpid->output_max))
    {
        hpid->p_out = -(hpid->output_max);
    }
}

// 输入滤波
static void F_Smooth_Target(digital_PID_t *hpid)
{
    // TODO:(math.h)
    hpid->ref_p = ramp_calc(hpid->ramp_target, hpid->target);
}

// 输出滤波
static void F_Output_Filter(digital_PID_t *hpid)
{
    hpid->output = hpid->output * (1 - hpid->output_LPF_RC) + hpid->output_last * hpid->output_LPF_RC;
}

static void F_Error_Handler(digital_PID_t *hpid)
{
    if (hpid->output < hpid->output_max * 0.001f || fabsf(hpid->target) < 0.0001f)
    {
        return;
    }
    if ((fabsf(hpid->target - hpid->measure) / fabsf(hpid->target)) > 0.95f)
    {
        // Motor blocked counting
        hpid->error_cnt++;
    }
    else
    {
        hpid->error_cnt = 0;
    }

    if (hpid->error_cnt > 500)
    {
        // Motor blocked over 1000times
        hpid->error_flag = 1;
    }
}
