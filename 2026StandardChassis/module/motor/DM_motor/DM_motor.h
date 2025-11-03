/**
* @file DM_motor.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__

#include "drv_motor.h"

#include "defense_center.h"

#define DM_MOTOR_CNT 4

#define DM_MODE_SETTING 0

#define MIT_MODE 	0x000
#define POS_MODE	0x100
#define SPEED_MODE	0x200
#define DM124_MODE	0x300

#define DM_P_MIN  (-3.141593)
#define DM_P_MAX  3.141593
#define DM_V_MIN  (-10.0f)
#define DM_V_MAX  10.0f
#define DM_T_MIN  (-10.0f)
#define DM_T_MAX   10.0f
#define DM_KP_MIN 0.0f
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0.0f
#define DM_KD_MAX 5.0f

#define SPEED_RAMP_COEF 0.85f

typedef enum
{
	DM_ERROR_NONE            = 0x00U,
	DM_MOTOR_BLOCKED_ERROR   = 0x01U,
	DM_MOTOR_LOST_ERROR      = 0x02U,
	DM_MOTOR_SUPERLOAD_ERROR = 0x03U,
} DM_error_e;

typedef struct
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float kp;
	float ki;
	float velocity;
	float position;
	float last_position;
	float torque;
	float t_mos;
	float t_rotor;
	int32_t total_round;
	float dm_diff;
} DM_motor_callback_t;

typedef struct
{
	float position_des;
	float velocity_des;
	float torque_des;
	float Kp;
	float Kd;
} DM_motor_fillmessage_t;

typedef enum
{
	DM_MOTOR_DIFF     = 0,
	DM_MOTOR_ABSOLUTE = 1,
} DM_motor_feedback_data_e;

typedef struct
{
    motor_type_e motor_type;        // 电机类型
    motor_reference_e motor_reference;

	motor_control_setting_t motor_settings; // 电机设置
	motor_controller_t motor_controller;    // 电机控制器

	CAN_instance_t *motor_can_instance;

	motor_working_type_e motor_state_flag; // 启停标志
    motor_error_detection_type_e motor_error_detection; // 异常检测

	DM_motor_callback_t receive_data;		// 电机反馈值
	DM_motor_fillmessage_t transmit_data;	// 电机目标值
    DM_motor_feedback_data_e motor_feedback;

	supervisor_t *supervisor;

	uint32_t feed_cnt;
	float dt;

	uint8_t dm_tx_id;
	uint8_t dm_rx_id;
	uint16_t dm_mode;
	uint16_t contorl_mode_state;
	float dm_offset_control;
	DM_error_e error_code;
	uint32_t error_beat;
} DM_motor_t;

typedef enum
{
	DM_CMD_ENABLE_MODE   = 0xFC, // 使能,会响应指令
	DM_CMD_DISABLE_MODE  = 0xFD, // 停止
	DM_CMD_ZERO_POSITION = 0xFE, // 将当前的位置设置为编码器零位
	DM_CMD_CLEAR_ERROR   = 0xFB 	// 清除电机过热错误
} DM_motor_mode_e;

void DM_Motor_Set_Zeropoint(DM_motor_t *motor);

void DM_Motor_Start(DM_motor_t *motor);

void DM_Motor_Stop(DM_motor_t *motor);

void DM_Motor_Clear_Error(DM_motor_t *motor);

void DM_Motor_SetTar(DM_motor_t *motor, float val);

void DM_Motor_Control(void);

void DM_MIT_Ctrl(DM_motor_t *motor,
                 float pos,
                 float vel,
                 float kp,
                 float kd,
                 float torq);

void DM_Pos_Speed_Ctrl(DM_motor_t *motor, float pos, float vel);

void DM_Speed_Ctrl(DM_motor_t *motor, float vel);

void DM_DM124_Ctrl(DM_motor_t *motor, float pid_ref);

void DM_Motor_ENABLE(DM_motor_t *motor);

void DM_Motor_DISABLE(DM_motor_t *motor);

uint8_t DM_Motor_Error_Judge(DM_motor_t *motor);

DM_motor_t *DM_Motor_Init(motor_init_config_t *config);

#endif /* __DM_MOTOR_H__ */
