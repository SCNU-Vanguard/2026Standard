/**
******************************************************************************
 * @file    kalman_one_filter.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>
#include "kalman_one_filter.h"

/*
	测量噪声大 R需要设置的较大
	模型不准确 Q需要增大
	R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值(Q/R取0无意义)
	Q/R决定了滤波器的稳态频率响应。Q增大和R减小都会导致卡尔曼增益K变大
	卡尔曼增益的值越大，意味着越相信测量值的输出，从而使得估计值倾向于测量值，，收敛速度越快，最优估计值震荡越明显
	A B H 根据模型来确定
	P变大也会导致卡尔曼增益变大，但是很快会迭代至稳定值，因此P的初始值只会对前几个周期的估计值有影响。
 */

/*****************  一阶卡尔曼  *****************/

/**
 * @brief			一阶卡尔曼滤波初始化
 * @param[out]		state : 滤波结构数据指针
 * @param[in]		q & r
 */
void Kalman_One_Init(kalman_one_filter_t *state, float q, float r)
{
    state->x = 0.0f;
    state->p = 0.0f;
    state->A = 1.0f;
    state->H = 1.0f;
    state->q = q;
    state->r = r;
}

/**
 * @brief			一阶卡尔曼滤波
 * @param[out]		state : 滤波结构数据指针
 * @param[in]		z_measure : 原始数据
 */
float Kalman_One_Filter(kalman_one_filter_t *state, float z_measure)
{
    /* Predict */
	// 时间更新(预测): X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k)
    state->x = state->A * state->x;
    // 更新先验协方差: P(k|k-1) = A(k,k-1)*A(k,k-1)^T*P(k-1|k-1)+Q(k)
    state->p = state->A * state->A * state->p + state->q;

    /* Measurement */
    // 计算卡尔曼增益: K(k) = P(k|k-1)*H(k)^T/(P(k|k-1)*H(k)*H(k)^T + R(k))
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    // 测量更新(校正): X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1))
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    // 更新后验协方差: P(k|k) =（I-K(k)*H(k))*P(k|k-1)
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}