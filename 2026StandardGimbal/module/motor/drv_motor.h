/**
 * @file drv_motor.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __DRV_MOTOR_H__
#define __DRV_MOTOR_H__

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "pid.h"

#include "bsp_can.h"

#ifndef PI
#define PI 3.1415926535f
#endif
#define PI2 1.5707963268f // 2 pi

#define RAD_2_DEGREE 57.2957795f    // 180/pi
#define DEGREE_2_RAD 0.01745329252f // pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f       // ×360°/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f // ×2pi/60sec

typedef enum
{
	ABS  = 0,
	INCR = 1
} motor_reference_e;

/* 电机控制器,包括其他来源的反馈数据指针,3环控制器和电机的参考输入*/
// 后续增加前馈数据指针
typedef struct
{
	float *other_angle_feedback_ptr; // 其他反馈来源的角度反馈数据指针
	float *other_speed_feedback_ptr;// 其他反馈来源的速度反馈数据指针

	float *angle_feedforward_ptr; // 角度前馈数据指针
	float *speed_feedforward_ptr; // 速度前馈数据指针
	float *current_feedforward_ptr; // 电流前馈数据指针
	float *torque_feedforward_ptr; // 扭矩前馈数据指针

	PID_t *angle_PID;
	PID_t *speed_PID;
	PID_t *current_PID;
	PID_t *torque_PID;

	float pid_ref; // 将会作为每个环的输入和输出顺次通过串级闭环
} motor_controller_t;

/**
 * @brief 闭环类型,如果需要多个闭环,则使用或运算
 *        例如需要速度环和电流环: torque_LOOP|SPEED_LOOP
 */
typedef enum
{
	OPEN_LOOP    = 0b0000,
	TORQUE_LOOP  = 0b0001,
	SPEED_LOOP   = 0b0010,
	ANGLE_LOOP   = 0b0100,
	CURRENT_LOOP = 0b1000,

	// only for checking 本框架电流环一般不考虑
	SPEED_AND_TORQUE_LOOP  = 0b0011,
	ANGLE_AND_SPEED_LOOP   = 0b0110,
	ALL_THREE_LOOP         = 0b0111,
} closeloop_type_e;

typedef enum
{
	FEEDFORWARD_NONE             = 0b00,
	TORQUE_FEEDFORWARD           = 0b01,
	SPEED_FEEDFORWARD            = 0b10,
	TORQUE_AND_SPEED_FEEDFORWARD = TORQUE_FEEDFORWARD | SPEED_FEEDFORWARD,
} feedfoward_type_e;

/* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针,详见Motor_Controller_s*/
typedef enum
{
	MOTOR_FEED = 0,
	OTHER_FEED = 1,
} feedback_type_e;

/* 电机正反转标志 */
typedef enum
{
	MOTOR_DIRECTION_NORMAL  = 0,
	MOTOR_DIRECTION_REVERSE = 1
} motion_reverse_flag_e;

/* 反馈量正反标志 */
typedef enum
{
	FEEDBACK_DIRECTION_NORMAL  = 0,
	FEEDBACK_DIRECTION_REVERSE = 1
} feedback_reverse_flag_e;

typedef enum
{
	MOTOR_DISABLE = 0,
	MOTOR_ENABLE  = 1,
} motor_working_type_e;

typedef enum
{
	OTHER,
	GM6020,
	M3508,
	M2006,
	DM4310,
	DM8009P,
} motor_type_e;

typedef enum
{
	MOTOR_ANGLE,
	MOTOR_SPEED,
	MOTOR_IMU,
} motor_measure_e;

/* 异常检测 */
typedef enum
{
	MOTOR_ERROR_DETECTION_NONE  = 0b0000,
	MOTOR_ERROR_DETECTION_CRASH = 0b0001,
	MOTOR_ERROR_DETECTION_STUCK = 0b0010,
	MOTOR_ERROR_DETECTION_LOST  = 0b0100,
	MOTOR_ERROR_PROTECTED       = 0b1000,
} motor_error_detection_type_e;

/* 电机控制方式枚举 */
typedef enum
{
	TORQUE_LOOP_CONTRO = 0, //扭矩开环控制
	ANGLE_LOOP_CONTRO  = 1,  //位置闭环控制(由电机支持)
} motor_control_type_e;

/* 电机控制设置,包括闭环类型,反转标志和反馈来源 */
typedef struct
{
	closeloop_type_e outer_loop_type;    		// 最外层的闭环,未设置时默认为最高级的闭环
	closeloop_type_e close_loop_type;             // 使用几个闭环(串级)

	motion_reverse_flag_e motor_reverse_flag;             // 电机转向是否反转(不改变原数据)
	feedback_reverse_flag_e feedback_reverse_flag;        // 反馈数据是否反向(不改变原数据)

	feedback_type_e angle_feedback_source;       	// 角度反馈类型
	feedback_type_e speed_feedback_source;       	// 速度反馈类型

	feedfoward_type_e feedforward_flag;           // 前馈类型
} motor_control_setting_t;

/* 用于初始化CAN电机的结构体,各类电机通用 */
typedef struct
{
	motor_controller_t controller_param_init_config;
	motor_control_setting_t controller_setting_init_config;
	motor_type_e motor_type;
	can_init_config_t can_init_config;
	motor_control_type_e motor_control_type;  //控制类型
} motor_init_config_t;

#endif /* __DRV_MOTOR_H__ */
