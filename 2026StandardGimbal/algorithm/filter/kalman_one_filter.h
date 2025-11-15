/**
* @file kalman_one_filter.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __KALMAN_ONE_FILTER_H__
#define __KALMAN_ONE_FILTER_H__

#include <stdint.h>

typedef struct {
    float x;		// state
    float A;		// x(n)=A*x(n-1)+u(n),u(n)~N(0,q)
    float H;		// z(n)=H*x(n)+w(n),w(n)~N(0,r)
    float q;		// process(predict) noise convariance
    float r;		// measure noise(error) convariance
    float p;		// estimated error convariance
    float gain;		// kalman gain
} kalman_one_filter_t;

extern void Kalman_One_Init(kalman_one_filter_t *state, float q, float r);
extern float Kalman_One_Filter(kalman_one_filter_t *state, float z_measure);

#endif /* __KALMAN_ONE_FILTER_H__ */