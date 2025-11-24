/**
******************************************************************************
* @file    shoot.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>

#include "shoot.h"
#include "chassis.h"

uint16_t target_shoot_frequence = 0;

PID_t chassis_2006_speed_pid = {
    .kp = 1.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .output_limit = 10000.0f,
    .integral_limit = 10000.0f,
    .dead_band = 0.0f,
};

motor_init_config_t chassis_2006_init = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_2006_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = M2006,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x05,
        .rx_id = 0x05,
    },

    .motor_control_type = TORQUE_LOOP_CONTRO,
};

DJI_motor_instance_t *chassis_shoot_motor;

void Shoot_Init(void)
{
    chassis_shoot_motor = DJI_Motor_Init(&chassis_2006_init);
}

void Shoot_Enable(void)
{
    DJI_Motor_Enable(chassis_shoot_motor);
}

void Shoot_Stop(void)
{
    DJI_Motor_Stop(chassis_shoot_motor);
}

void Shoot_State_Machine(void)
{
    static uint16_t init_count = 0;
    uint8_t shoot_mode = uart2_rx_message.shoot_mode;
    if (init_count < 1000)
    {
        init_count++;
        shoot_mode = SHOOT_MODE_STOP;
    }
    if (chassis_mode)
    {
        switch (shoot_mode)
        {
        case SHOOT_MODE_STOP:
            Shoot_Stop();
            break;
        case SHOOT_MODE_FIRE:
            Shoot_Enable();
            DJI_Motor_Set_Ref(chassis_shoot_motor, target_shoot_frequence);
            break;
        default:
            Shoot_Stop();
            break;
        }
    }
    else
    {
        Shoot_Stop();
    }
}
