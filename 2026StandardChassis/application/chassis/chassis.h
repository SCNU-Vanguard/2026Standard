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

#define CHASSIS_MODE_MANUAL 1
#define CHASSIS_MODE_STOP 0

typedef struct 
{
    /* data */
}__attribute__((packed))chassis_behaviour_t;

typedef struct
{
    /* data */
}__attribute__((packed))chassis_cmd_t;

extern DJI_motor_instance_t *chassis_motor_drive_1;
extern DJI_motor_instance_t *chassis_motor_drive_2;
extern DJI_motor_instance_t *chassis_motor_drive_3;
extern DJI_motor_instance_t *chassis_motor_drive_4;

extern DJI_motor_instance_t *chassis_motor_direct_1;
extern DJI_motor_instance_t *chassis_motor_direct_2;
extern DJI_motor_instance_t *chassis_motor_direct_3;
extern DJI_motor_instance_t *chassis_motor_direct_4;

extern DM_motor_t *chassis_4310_instance;

extern float target_angle_yaw;

void Chassis_Init(void);
void Chassis_Enable(void);
void Chassis_Stop(void);
void Chassis_Resolving(float x_speed, float y_speed, float omega_speed, float gimbal_angle_yaw_offset);
float Chassis_Get_Actual_Omega(void);
void Chassis_State_Machine(void);

#endif /* __CHASSIS_H__ */