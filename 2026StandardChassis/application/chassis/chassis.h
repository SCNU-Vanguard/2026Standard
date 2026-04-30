/**
 * @file chassis.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include <stdint.h>
#include "DJI_motor.h"
#include "DM_motor.h"
#include "rs485.h"

#define CHASSIS_MODE_SEMIAUTO 3
#define CHASSIS_MODE_AUTO 2
#define CHASSIS_MODE_MANUAL 1
#define CHASSIS_MODE_STOP 0

typedef struct
{
    /* data */
} __attribute__((packed)) chassis_behaviour_t;

typedef struct
{
    /* data */
} __attribute__((packed)) chassis_cmd_t;

typedef struct
{
    float last_speed;   // 上一次的输出值
    float accel_step; // 升速步长（加速度控制）
    float decel_step; // 降速步长（通常降速可以设得比升速快一些，保证安全）
} Speed_Ramp_t;

extern DJI_motor_instance_t *chassis_motor_drive_1;
extern DJI_motor_instance_t *chassis_motor_drive_2;
extern DJI_motor_instance_t *chassis_motor_drive_3;
extern DJI_motor_instance_t *chassis_motor_drive_4;

extern DJI_motor_instance_t *chassis_motor_direct_1;
extern DJI_motor_instance_t *chassis_motor_direct_2;
extern DJI_motor_instance_t *chassis_motor_direct_3;
extern DJI_motor_instance_t *chassis_motor_direct_4;

extern DM_motor_t *gimbal_motor_yaw;

extern float target_angle_yaw;
extern float target_angle_yaw_temp;
extern uint8_t chassis_mode;
extern float target_speed;
extern float yaw_zero_offset;
extern float gimbal_angle_yaw_motor2imu;
void Chassis_Init(void);
void Chassis_Enable(void);
void Chassis_Stop(void);
void Chassis_Resolving(float x_speed, float y_speed, float omega_speed, float gimbal_angle_yaw_offset);
float Chassis_Get_Actual_Omega(void);
void Chassis_State_Machine(void);

#endif /* __CHASSIS_H__ */