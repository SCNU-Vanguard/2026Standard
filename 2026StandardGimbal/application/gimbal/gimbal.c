/**
******************************************************************************
* @file    gimbal.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>

#include "gimbal_task.h"
#include "gimbal.h"

#include "message_center.h"

#include "DJI_motor.h"
#include "DM_motor.h"
#include "remote_control.h"

uint8_t gimbal_mode = 0;
uint8_t gimbal_mode_last = 0;
float target_angle_pitch = 0;
float target_angle_yaw = 0;
uint16_t init_count = 0;

PID_t gimbal_6020_angle_pid = {
    .kp = 12.0f,
    .ki = 0.0f,
    .kd = 600.0f,
    .output_limit = 12.0f,
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
};

PID_t gimbal_6020_speed_pid = {
    .kp = 600.0f,
    .ki = 30.0f,
    .kd = 0.0f,
    .output_limit = 25000.0f,
    .integral_limit = 25000.0f,
    .dead_band = 0.0f,
};

PID_t gimbal_4310_angle_pid = {
    .kp = 30.0f,
    .ki = 0.0f,
    .kd = 600.0f,
    .output_limit = 5.0f,
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
};

PID_t gimbal_4310_speed_pid = {
    .kp = 1.0f,
    .ki = 0.002f,
    .kd = 0.0f,
    .output_limit = 10.0f,
    .integral_limit = 10.0f,
    .dead_band = 0.0f,
};

motor_init_config_t gimbal_6020_init = {
    .controller_param_init_config = {
        .angle_PID = &gimbal_6020_angle_pid,
        .speed_PID = &gimbal_6020_speed_pid,
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
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_AND_SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = GM6020,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x01,
        .rx_id = 0x01,
    },
};

motor_init_config_t gimbal_4310_init = {
    .controller_param_init_config = {
        .angle_PID = &gimbal_4310_angle_pid,
        .speed_PID = &gimbal_4310_speed_pid,
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
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_AND_SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = DM4310,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x01,
        .rx_id = 0x11,
    },
};

DJI_motor_instance_t *gimbal_motor_pitch;
DM_motor_t *gimbal_motor_yaw;

void Gimbal_Init(void)
{
    gimbal_motor_pitch = DJI_Motor_Init(&gimbal_6020_init);
    gimbal_motor_yaw = DM_Motor_Init(&gimbal_4310_init);
}

void Get_Gimbal_Mode(void)
{
    if (switch_is_up(rc_data->rc.switch_left))
    {
        gimbal_mode = GIMBAL_MODE_AUTO;
    }
    else if (switch_is_mid(rc_data->rc.switch_left))
    {
        gimbal_mode = GIMBAL_MODE_MANUAL;
    }
    else if (switch_is_down(rc_data->rc.switch_left))
    {
        gimbal_mode = GIMBAL_MODE_STOP;
    }
    else
    {
        gimbal_mode = GIMBAL_MODE_STOP;
    }
}

void Chassis_Control(void)
{
    uart2_tx_message.chassis_mode = (gimbal_mode == GIMBAL_MODE_MANUAL);
    uart2_tx_message.delta_target_angle_yaw = -rc_data->rc.rocker_r_;
    uart2_tx_message.target_x_speed = rc_data->rc.rocker_l1;
    uart2_tx_message.target_y_speed = rc_data->rc.rocker_l_;
    if (rc_data->rc.dial)
    {
        uart2_tx_message.target_omega_speed = 1.0f;
    }
    else
    {
        uart2_tx_message.target_omega_speed = 0.0f;
    }
}

void Gimbal_State_Machine(void)
{
    if (init_count < 1000)
    {
        init_count++;
        gimbal_mode = GIMBAL_MODE_STOP;
    }
    switch (gimbal_mode)
    {
    case GIMBAL_MODE_AUTO:
        target_angle_pitch = gimbal_motor_pitch->measure.rad;
        DJI_Motor_Stop(gimbal_motor_pitch);
        // DM_Motor_Stop(gimbal_motor_yaw);
        // DM_Motor_DISABLE(gimbal_motor_yaw);
        gimbal_mode_last = GIMBAL_MODE_AUTO;
        break;

    case GIMBAL_MODE_MANUAL:
        // target_angle_yaw = target_angle_yaw - rc_data->rc.rocker_r_ * 0.00001f;
        target_angle_pitch = target_angle_pitch + rc_data->rc.rocker_r1 * 0.000005f;
        target_angle_pitch = Value_Limit(target_angle_pitch, 4.80, 5.70);

        DJI_Motor_Set_Ref(gimbal_motor_pitch, target_angle_pitch);
        // DM_Motor_SetTar(gimbal_motor_yaw, target_angle_yaw);

        DJI_Motor_Enable(gimbal_motor_pitch);
        // DM_Motor_Start(gimbal_motor_yaw);
        // DM_Motor_ENABLE(gimbal_motor_yaw);
        DJI_Motor_Control();
        // DM_Motor_Control();
        gimbal_mode_last = GIMBAL_MODE_MANUAL;
        break;

    case GIMBAL_MODE_STOP:
        target_angle_pitch = gimbal_motor_pitch->measure.rad;
        DJI_Motor_Stop(gimbal_motor_pitch);
        // DM_Motor_Stop(gimbal_motor_yaw);
        // DM_Motor_DISABLE(gimbal_motor_yaw);
        gimbal_mode_last = GIMBAL_MODE_STOP;
        break;

    default:
        target_angle_pitch = gimbal_motor_pitch->measure.rad;
        DJI_Motor_Stop(gimbal_motor_pitch);
        // DM_Motor_Stop(gimbal_motor_yaw);
        // DM_Motor_DISABLE(gimbal_motor_yaw);
        gimbal_mode_last = GIMBAL_MODE_STOP;
        break;
    }
}

void Remote_Deadzone_Control(void)
{
    if (abs(rc_data->rc.rocker_r_) <= 66)
    {
        rc_data->rc.rocker_r_ = 0;
    }
    if (abs(rc_data->rc.rocker_r1) <= 66)
    {
        rc_data->rc.rocker_r1 = 0;
    }
    if (abs(rc_data->rc.rocker_l_) <= 66)
    {
        rc_data->rc.rocker_l_ = 0;
    }
    if (abs(rc_data->rc.rocker_l1) <= 66)
    {
        rc_data->rc.rocker_l1 = 0;
    }
    if (abs(rc_data->rc.dial) <= 659)
    {
        rc_data->rc.dial = 0;
    }
}
