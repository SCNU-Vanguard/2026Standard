/**
******************************************************************************
 * @file    mahony_filter.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "mahony_filter.h"

/*-----------------------MahonyAHRS滤波器----------------------*/
/*-----------------------------------------------------------*/
mahony_filter_t mahony_filter;

/*计算旋转矩阵*/	//大地坐标系 R 转换到机体坐标系 b 的坐标转换矩阵
void Mahony_Filter_RotationMatrix_Update(mahony_filter_t *mahony_filter)
{
  float q1q1 = mahony_filter->q1 * mahony_filter->q1;
  float q2q2 = mahony_filter->q2 * mahony_filter->q2;
  float q3q3 = mahony_filter->q3 * mahony_filter->q3;

  float q0q1 = mahony_filter->q0 * mahony_filter->q1;
  float q0q2 = mahony_filter->q0 * mahony_filter->q2;
  float q0q3 = mahony_filter->q0 * mahony_filter->q3;
  float q1q2 = mahony_filter->q1 * mahony_filter->q2;
  float q1q3 = mahony_filter->q1 * mahony_filter->q3;
  float q2q3 = mahony_filter->q2 * mahony_filter->q3;

  mahony_filter->rotation_matrix[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
  mahony_filter->rotation_matrix[0][1] = 2.0f * (q1q2 - q0q3);
  mahony_filter->rotation_matrix[0][2] = 2.0f * (q1q3 + q0q2);

  mahony_filter->rotation_matrix[1][0] = 2.0f * (q1q2 + q0q3);
  mahony_filter->rotation_matrix[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
  mahony_filter->rotation_matrix[1][2] = 2.0f * (q2q3 - q0q1);

  mahony_filter->rotation_matrix[2][0] = 2.0f * (q1q3 - q0q2);
  mahony_filter->rotation_matrix[2][1] = 2.0f * (q2q3 + q0q1);
  mahony_filter->rotation_matrix[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

// 将陀螺仪和加速度计的输入值赋予滤波器结构体
void Mahony_Filter_Input(mahony_filter_t *mahony_filter, axis_3f_t gyro, axis_3f_t acc)
{
  mahony_filter->gyro = gyro;
  mahony_filter->acc = acc;
}

// 使用Mahony算法更新滤波器状态
void Mahony_Filter_Update(mahony_filter_t *mahony_filter)
{
  float normalise;
  float ex, ey, ez;

  /*角速度，度转弧度*/
//    mahony_filter->gyro.x *= DEG2RAD;        /* 度转弧度 */
//    mahony_filter->gyro.y *= DEG2RAD;
//    mahony_filter->gyro.z *= DEG2RAD;;
  /*单位化加速计测量值*/
  normalise = sqrt(mahony_filter->acc.x * mahony_filter->acc.x + mahony_filter->acc.y * mahony_filter->acc.y + mahony_filter->acc.z * mahony_filter->acc.z);

  mahony_filter->acc.x = mahony_filter->acc.x / normalise;
  mahony_filter->acc.y = mahony_filter->acc.y / normalise;
  mahony_filter->acc.z = mahony_filter->acc.z / normalise;

  /*加速计读取的方向与重力加速计方向的差值，用向量叉乘计算*/
  //vx=mahony_filter->rotation_matrix[2][0]=2.0f * (q1q3 + -q0q2)
  //vy=mahony_filter->rotation_matrix[2][1]=2.0f * (q2q3 - -q0q1)
  //vz=mahony_filter->rotation_matrix[2][2]=1.0f - 2.0f * q1q1 - 2.0f * q2q2
  ex = (mahony_filter->acc.y * mahony_filter->rotation_matrix[2][2] - mahony_filter->acc.z * mahony_filter->rotation_matrix[2][1]);
  ey = (mahony_filter->acc.z * mahony_filter->rotation_matrix[2][0] - mahony_filter->acc.x * mahony_filter->rotation_matrix[2][2]);
  ez = (mahony_filter->acc.x * mahony_filter->rotation_matrix[2][1] - mahony_filter->acc.y * mahony_filter->rotation_matrix[2][0]);

  /*误差累计，与积分常数相乘*/
  mahony_filter->ex_intergral += mahony_filter->Ki * ex * mahony_filter->dt;
  mahony_filter->ey_intergral += mahony_filter->Ki * ey * mahony_filter->dt;
  mahony_filter->ez_intergral += mahony_filter->Ki * ez * mahony_filter->dt;

  /*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
  mahony_filter->gyro.x += mahony_filter->Kp * ex + mahony_filter->ex_intergral;
  mahony_filter->gyro.y += mahony_filter->Kp * ey + mahony_filter->ey_intergral;
  mahony_filter->gyro.z += mahony_filter->Kp * ez + mahony_filter->ez_intergral;

  /* 一阶近似算法，四元数运动学方程的离散化形式和积分 */
  float q0Last = mahony_filter->q0;
  float q1Last = mahony_filter->q1;
  float q2Last = mahony_filter->q2;
  float q3Last = mahony_filter->q3;
  float halfT = mahony_filter->dt * 0.5f;
  mahony_filter->q0 += (-q1Last * mahony_filter->gyro.x - q2Last * mahony_filter->gyro.y - q3Last * mahony_filter->gyro.z) * halfT;
  mahony_filter->q1 += (q0Last * mahony_filter->gyro.x + q2Last * mahony_filter->gyro.z - q3Last * mahony_filter->gyro.y) * halfT;
  mahony_filter->q2 += (q0Last * mahony_filter->gyro.y - q1Last * mahony_filter->gyro.z + q3Last * mahony_filter->gyro.x) * halfT;
  mahony_filter->q3 += (q0Last * mahony_filter->gyro.z + q1Last * mahony_filter->gyro.y - q2Last * mahony_filter->gyro.x) * halfT;

  /*单位化四元数*/
  normalise = sqrt(mahony_filter->q0 * mahony_filter->q0 + mahony_filter->q1 * mahony_filter->q1 + mahony_filter->q2 * mahony_filter->q2 + mahony_filter->q3 * mahony_filter->q3);

  mahony_filter->q0 = mahony_filter->q0 / normalise;
  mahony_filter->q1 = mahony_filter->q1 / normalise;
  mahony_filter->q2 = mahony_filter->q2 / normalise;
  mahony_filter->q3 = mahony_filter->q3 / normalise;

  /*计算旋转矩阵*/
  mahony_filter->Mahony_Filter_RotationMatrix_Update(mahony_filter);
}

// 从旋转矩阵中提取姿态角
void Mahony_Filter_Output(mahony_filter_t *mahony_filter)
{
  /*计算roll pitch yaw 欧拉角*/
  mahony_filter->pitch = -asinf(mahony_filter->rotation_matrix[2][0]);
  mahony_filter->roll = atan2f(mahony_filter->rotation_matrix[2][1],
			       mahony_filter->rotation_matrix[2][2]);
  mahony_filter->yaw = atan2f(mahony_filter->rotation_matrix[1][0],
			      mahony_filter->rotation_matrix[0][0]);
}

// 初始化Mahony滤波器的参数和函数指针
void Mahony_Filter_Init(mahony_filter_t *mahony_filter, float Kp, float Ki, float dt)
{
  mahony_filter->Kp = Kp;
  mahony_filter->Ki = Ki;
  mahony_filter->dt = dt;
  mahony_filter->q0 = 1;
  mahony_filter->Mahony_Filter_Input = Mahony_Filter_Input;
  mahony_filter->Mahony_Filter_Update = Mahony_Filter_Update;
  mahony_filter->Mahony_Filter_Output = Mahony_Filter_Output;
  mahony_filter->Mahony_Filter_RotationMatrix_Update = Mahony_Filter_RotationMatrix_Update;
}
/*-----------------------MahonyAHRS滤波器----------------------*/
/*-----------------------------------------------------------*/