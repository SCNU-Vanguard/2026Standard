/**
******************************************************************************
 * @file    INS.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

#include "INS.h"

#include "bmi088.h"

#include "bsp_dwt.h"

#include "QuaternionEKF.h"
#include "mahony_filter.h"
#include "user_lib.h"

#include "BMI088driver.h"

INS_behaviour_t INS;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

bmi088_data_t imu_data;

axis_3f_t gyro_ins, accel_ins;

static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);

static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

// float imu_time;

void INS_Data_Update(void)
{
	// BMI088_Read(&BMI088);
	BMI088_Read_All(bmi088_h7, &imu_data);

	/******************************测试获取时间*****************************/

	// TIME_ELAPSE(imu_time, BMI088_Read_All(bmi088_h7, &imu_data););

	/******************************测试获取时间*****************************/
}

static void EKF_Quaternion_Init(float *init_q4)
{
	float acc_init[3]     = {0};
	float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
	float axis_rot[3]     = {0};           				// 旋转轴
	// 读取100次加速度计数据,取平均值作为初始值

	for (uint8_t i = 0 ; i < 100 ; ++i)
	{
		INS_Data_Update( );
		acc_init[IMU_X] += imu_data.acc[IMU_X];
		acc_init[IMU_Y] += imu_data.acc[IMU_Y];
		acc_init[IMU_Z] += imu_data.acc[IMU_Z];
		DWT_Delay(0.001);
	}
	for (uint8_t i = 0 ; i < 3 ; ++i)
	{
		acc_init[i] /= 100;
	}

	Norm3d(acc_init);
	// 计算原始加速度矢量和导航系重力加速度矢量的夹角
	float angle = acosf(Dot3d(acc_init, gravity_norm));
	Cross3d(acc_init, gravity_norm, axis_rot);
	Norm3d(axis_rot);

	init_q4[0] = cosf(angle / 2.0f);
	for (uint8_t i = 0 ; i < 2 ; ++i)
	{
		init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
	}
}

void INS_Init(void)
{
	float init_quaternion[4] = {1, 0, 0, 0};
	EKF_Quaternion_Init(init_quaternion);
	// IMU_QuaternionEKF_Init(init_quaternion, 12, 0.005f, 1000000 * 15, 0.9998f, 0.005f);
	IMU_QuaternionEKF_Init(init_quaternion, 12, 0.005f, 1000000 * 20, 0.9999f, 0.005f);

	INS.AccelLPF = 0.01f; // 加速度低通滤波系数
}

const float gravity[3] = {0, 0, 9.81f};
int stop_time;
float ins_time = 0.0f;

void INS_Calculate(float dt)
{
	INS_Data_Update( );

	INS.Gyro[IMU_X] = imu_data.gyro[IMU_X];
	INS.Gyro[IMU_Y] = imu_data.gyro[IMU_Y];
	INS.Gyro[IMU_Z] = imu_data.gyro[IMU_Z];

	gyro_ins.x = imu_data.gyro[IMU_X];
	gyro_ins.y = imu_data.gyro[IMU_Y];
	gyro_ins.z = imu_data.gyro[IMU_Z];

	INS.Accel[IMU_X] = imu_data.acc[IMU_X];
	INS.Accel[IMU_Y] = imu_data.acc[IMU_Y];
	INS.Accel[IMU_Z] = imu_data.acc[IMU_Z];

	accel_ins.x = imu_data.acc[IMU_X];
	accel_ins.y = imu_data.acc[IMU_Y];
	accel_ins.z = imu_data.acc[IMU_Z];

	IMU_QuaternionEKF_Update(INS.Gyro[IMU_X],
	                         INS.Gyro[IMU_Y],
	                         INS.Gyro[IMU_Z],
	                         INS.Accel[IMU_X],
	                         INS.Accel[IMU_Y],
	                         INS.Accel[IMU_Z],
	                         dt);

	memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));
	BodyFrameToEarthFrame(xb, INS.xn, INS.q);
	BodyFrameToEarthFrame(yb, INS.yn, INS.q);
	BodyFrameToEarthFrame(zb, INS.zn, INS.q);

	// INS.q[0] = mahony.q0 * QEKF_INS.accLPFcoef + QEKF_INS.q[0] * (1 - QEKF_INS.accLPFcoef);
	// INS.q[1] = mahony.q1 * QEKF_INS.accLPFcoef + QEKF_INS.q[1] * (1 - QEKF_INS.accLPFcoef);
	// INS.q[2] = mahony.q2 * QEKF_INS.accLPFcoef + QEKF_INS.q[2] * (1 - QEKF_INS.accLPFcoef);
	// INS.q[3] = mahony.q3 * QEKF_INS.accLPFcoef + QEKF_INS.q[3] * (1 - QEKF_INS.accLPFcoef);  

	float gravity_b[3];
	EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
	for (uint8_t i = 0 ; i < 3 ; i++) // 同样过一个低通滤波
	{
		INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
	}
	BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

	// 死区处理
	if (fabsf(INS.MotionAccel_n[0]) < 0.025f)
	{
		INS.MotionAccel_n[0] = 0.0f; // x轴
	}
	if (fabsf(INS.MotionAccel_n[1]) < 0.025f)
	{
		INS.MotionAccel_n[1] = 0.0f; // y轴
	}
	if (fabsf(INS.MotionAccel_n[2]) < 0.045f)
	{
		INS.MotionAccel_n[2] = 0.0f; // z轴
		stop_time++;
	}
	if (stop_time > 10)
	{ //静止10ms
		stop_time = 0;
		INS.v_n   = 0.0f;
	}

	if (ins_time > 3000.0f)
	{
		INS.v_n      = INS.v_n + INS.MotionAccel_n[1] * 0.001f;
		INS.x_n      = INS.x_n + INS.v_n * 0.001f;
		INS.ins_flag = 1; // 四元数基本收敛，加速度也基本收敛，可以开始底盘任务
		// 获取最终数据
		//      INS.Pitch = mahony.roll;
		//      INS.Roll = mahony.pitch;
		//      INS.Yaw = mahony.yaw;
		INS.Yaw           = QEKF_INS.Yaw;
		INS.Pitch         = QEKF_INS.Pitch;
		INS.Roll          = QEKF_INS.Roll;
		INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
		// INS.YawTotalAngle=INS.YawTotalAngle+INS.Gyro[2]*0.001f;

		if (INS.Yaw - INS.YawAngleLast > 3.1415926f)
		{
			INS.YawRoundCount--;
		}
		else if (INS.Yaw - INS.YawAngleLast < -3.1415926f)
		{
			INS.YawRoundCount++;
		}
		INS.YawTotalAngle = 6.283f * INS.YawRoundCount + INS.Yaw;
		INS.YawAngleLast  = INS.Yaw;
	}
	else
	{
		ins_time++;
	}
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
	vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] + (q[1] * q[2] - q[0] * q[3]) * vecBF[1] + (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

	vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] + (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

	vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] + (q[2] * q[3] + q[0] * q[1]) * vecBF[1] + (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
	vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] + (q[1] * q[2] + q[0] * q[3]) * vecEF[1] + (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

	vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] + (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

	vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] + (q[2] * q[3] - q[0] * q[1]) * vecEF[1] + (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
