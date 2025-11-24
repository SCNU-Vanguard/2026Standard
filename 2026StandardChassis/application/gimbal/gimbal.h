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

#define GIMBAL_MODE_AUTO 2
#define GIMBAL_MODE_MANUAL 1
#define GIMBAL_MODE_STOP 0

typedef struct 
{
    /* data */
}__attribute__((packed))gimbal_behaviour_t;

typedef struct
{
    /* data */
}__attribute__((packed))gimbal_cmd_t;    

// extern DJI_motor_instance_t *gimbal_motor_pitch;
// extern DM_motor_t *gimbal_motor_yaw;
extern uint8_t gimbal_mode;
// extern uint8_t gimbal_mode_last;
// extern float target_angle_pitch;
// extern float target_angle_yaw;

// void Gimbal_Init(void);
// void Get_Gimbal_Mode(void);
// void Gimbal_State_Machine(void);

#endif /* __GIMBAL_H__ */