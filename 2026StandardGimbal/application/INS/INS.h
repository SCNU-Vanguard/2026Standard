/**
* @file INS.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __INS_H__
#define __INS_H__

#include <stdint.h>

/*
 * 根据奈奎斯特——香农采样定理，采样频率至少为被采信号最高频率的2倍，
 * 实际应用一般选取最高频率的5~10倍数作为采样频率
*/

/*定义
 *通俗的讲，传感器的带宽是指传感器能够测量到的信号的最大变化率，
 *高于带宽的信号传感器无法通过采样还原真实的信号变化。
 *原理
 *传感器的测量通常包含噪声，以加速度测量为例，即使将传感器静止放置，
 *测量出来的值也会在平均值上下波动，为了降低噪声，传感器内部通常会自带一个低通滤波器（LPF）来过滤高频分量，
 *传感器的带宽就是指低通滤波器的带宽，高于带宽的信号变化将无法测量出来。
 *直观感受
 *直观的说，带宽设置越小，测量到的数据曲线越平滑，但信号变化会越滞后，对于高于带宽的信号变化无法测到。
 *相反，带宽设置越大，数据曲线的噪声越大，毛刺越多，但在测量数据变化的时候能够更加快速地响应其变化。
 *如何设置
 *带宽的设置需根据实际测量的物体的实际变化快慢来定，在能够满足测量要求的情况下应尽量设置低的带宽，
 *以获取更加平稳的测量数据，降低噪声，但需要高于需要测量的有效信号的变化频率。
*/

#define IMU_X 0
#define IMU_Y 1
#define IMU_Z 2

typedef struct
{
	float q[4]; // 四元数估计值

	float Gyro[3];  // 角速度
	float Accel[3]; // 加速度
	float MotionAccel_b[3]; // 机体坐标加速度
	float MotionAccel_n[3]; // 绝对系加速度

	float AccelLPF; // 加速度低通滤波系数

	// 加速度在绝对系的向量表示
	float xn[3];
	float yn[3];
	float zn[3];

	float atanxz;
	float atanyz;

	// 位姿
	float Roll;
	float Pitch;
	float Yaw;
	float YawTotalAngle;
	float YawAngleLast;
	float YawRoundCount;

	float v_n; //绝对系沿着水平运动方向的速度
	float x_n; //绝对系沿着水平运动方向的位移

	uint8_t ins_flag;
	/* data */
} INS_behaviour_t;

typedef struct
{
	/* data */
}__attribute__((packed)) INS_cmd_t;

extern INS_behaviour_t INS;

extern void INS_Data_Update(void);

extern void INS_Init(void);

extern void INS_Calculate(float dt);

#endif /* __INS_H__ */
