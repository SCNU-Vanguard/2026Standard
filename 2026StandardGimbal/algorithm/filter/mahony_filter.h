/**
* @file mahony_filter.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __MAHONY_FILTER_H__
#define __MAHONY_FILTER_H__

#include <stdint.h>

/*-----------------------MahonyAHRS滤波器----------------------*/
/*-----------------------------------------------------------*/
//比例项用于控制传感器的“可信度”，积分项用于消除静态误差。
//Kp越大，意味着通过加速度计得到误差后补偿越显著，即是越信任加速度计。
//反之 Kp越小时，加速度计对陀螺仪的补偿作用越弱，也就越信任陀螺仪。
//而积分项则用于消除角速度测量值中的有偏噪声，故对于经过零篇矫正的角速度测量值，一般选取很小的Ki。
//最后将补偿值补偿给角速度测量值，带入四元数微分方程中即可更新当前四元数。
typedef struct
{
  float x;
  float y;
  float z;
} axis_3f_t;

typedef struct MAHONY_FILTER_t
{
  // 输入参数
  float Kp, Ki;         // 比例和积分增益
  float dt;             // 采样时间间隔
  axis_3f_t gyro, acc;	// 陀螺仪和加速度计数据

  // 过程参数
  float ex_intergral, ey_intergral, ez_intergral;    // 积分误差累计
  float q0, q1, q2, q3;		// 四元数
  float rotation_matrix[3][3];  // 旋转矩阵

  // 输出参数
  float pitch, roll, yaw;       // 姿态角：俯仰角，滚转角，偏航角

  // 函数指针
  void (*Mahony_Filter_Init)(struct MAHONY_FILTER_t *mahony_filter,
		      float Kp,
		      float Ki,
		      float dt);
  void (*Mahony_Filter_Input)(struct MAHONY_FILTER_t *mahony_filter,
		       axis_3f_t gyro,
		       axis_3f_t acc);
  void (*Mahony_Filter_Update)(struct MAHONY_FILTER_t *mahony_filter);
  void (*Mahony_Filter_Output)(struct MAHONY_FILTER_t *mahony_filter);
  void (*Mahony_Filter_RotationMatrix_Update)(struct MAHONY_FILTER_t *mahony_filter);
} mahony_filter_t;

// 函数声明
void Mahony_Filter_Init(mahony_filter_t *mahony_filter, float Kp, float Ki, float dt); // 初始化函数
void Mahony_Filter_Input(mahony_filter_t *mahony_filter, axis_3f_t gyro, axis_3f_t acc); // 输入数据函数
void Mahony_Filter_Update(mahony_filter_t *mahony_filter);                   // 更新滤波器函数
void Mahony_Filter_Output(mahony_filter_t *mahony_filter);                   // 输出姿态角函数
void Mahony_Filter_RotationMatrix_Update(mahony_filter_t *mahony_filter);
/*-----------------------MahonyAHRS滤波器----------------------*/
/*-----------------------------------------------------------*/

#endif /* __MAHONY_FILTER_H__ */