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

/*****************  一阶卡尔曼  *****************/

/**
 * @brief			一阶卡尔曼滤波初始化
 * @param[out]		state : 滤波结构数据指针
 * @param[in]		q & r
 */
void Kalman_One_Init(kalman_onw_filter_t *state, float q, float r)
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
float Kalman_One_Filter(kalman_onw_filter_t *state, float z_measure)
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