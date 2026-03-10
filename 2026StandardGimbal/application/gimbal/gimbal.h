/**
* @file gimbal.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include <stdint.h>
#include "DJI_motor.h"
#include "DM_motor.h"
#include "remote_control.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "lpf.h"

#include "digital_pid.h"

#define GIMBAL_MODE_AUTO 2
#define GIMBAL_MODE_MANUAL 1
#define GIMBAL_MODE_STOP 0

#define CHASSIS_MODE_AUTO 2
#define CHASSIS_MODE_MANUAL 1
#define CHASSIS_MODE_STOP 0

typedef struct 
{
    /* data */
}__attribute__((packed))gimbal_behaviour_t;

typedef struct
{
    /* data */
}__attribute__((packed))gimbal_cmd_t;    

typedef enum
{
    GEAR_PRECISION,
    GEAR_NORMAL,
    GEAR_AGILE,
    SPEED_GEAR_COUNT,
}Speed_Gear_e;

extern DJI_motor_instance_t *gimbal_motor_pitch;
extern DM_motor_t *gimbal_motor_yaw;
extern Speed_Gear_e current_gear;

extern digital_PID_t gimbal_pitch_angle_digital_pid;
extern digital_PID_t gimbal_pitch_speed_digital_pid;

extern uint8_t gimbal_mode;
extern uint8_t gimbal_mode_last;
extern float target_angle_pitch;
extern float target_angle_yaw;
extern float target_angle_pitch_auto;
extern float target_angle_yaw_auto;
extern float angle_pitch_offset;
extern float angle_pitch_motor2imu;
extern float angle_yaw_motor2imu;
extern SemaphoreHandle_t g_xSemVPC;

void Gimbal_Init(void);
void Get_Gimbal_Mode(void);
void Gimbal_State_Machine(void);
void Chassis_Control(void);
void Remote_Deadzone_Control(void);
void Auto_Deadzone_Control(void);
#endif /* __GIMBAL_H__ */