/**
******************************************************************************
* @file    shoot.c
* @brief
* @author  miracle
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "shoot.h"
#include "gimbal.h"
#include "remote_control.h"
#include "rs485.h"
#include "shoot_motor.h"
#include "serial.h"
#include "Keyboard_Control.h"

#define FRICTION_WHEEL_RADIUS 0.025f    // 半径 25mm = 0.025m
#define FRICTION_SLIP_COEFFICIENT 0.85f // 经验打滑系数，摩擦轮与子弹间存在一定打滑，一般取0.8~0.9
#define FRICTION_MOTOR_RPM 8000.0f      // 摩擦轮电机转速 (最高为8000rpm，对应弹速约为23m/s)

#define PC_SHOOT_FIRE 2
#define PC_SHOOT_AIM 1
#define PC_SHOOT_STOP 0
uint8_t shoot_mode = 0;
uint8_t shoot_mode_last = 0;
uint8_t fire_ready_flag = 0;

uint32_t fric_pretime_cnt = 0;

float BULLET_V = 23.0f; // 目标弹速 (m/s)

PID_t gimbal_test_angle_pid = {
    .kp = 12.0f,
    .ki = 0.0f,
    .kd = 480.0f,
    .output_limit = 10.0f,
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
}; // 无效pid，纯粹填充用

PID_t gimbal_test_speed_pid = {
    .kp = 400.0f,
    .ki = 400.0f,
    .kd = 0.0f,
    .output_limit = 25000.0f,
    .integral_limit = 25000.0f,
    .dead_band = 0.0f,
}; // 无效pid，纯粹填充用

motor_init_config_t shoot_motor1_init = {
    .controller_param_init_config = {
        .angle_PID = &gimbal_test_angle_pid,
        .speed_PID = &gimbal_test_speed_pid,
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

    .motor_type = shoot,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x01,
        .rx_id = 0x011,
    },
};

motor_init_config_t shoot_motor2_init = {
    .controller_param_init_config = {
        .angle_PID = &gimbal_test_angle_pid,
        .speed_PID = &gimbal_test_speed_pid,
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

    .motor_type = shoot,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x02,
        .rx_id = 0x012,
    },
};

motor_init_config_t shoot_motor3_init = {
    .controller_param_init_config = {
        .angle_PID = &gimbal_test_angle_pid,
        .speed_PID = &gimbal_test_speed_pid,
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

    .motor_type = shoot,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x03,
        .rx_id = 0x013,
    },
};

/*正对枪管的情况下，左上为1，右上为2，正下为3*/
shoot_motor_instance_t *shoot_motor_1;
shoot_motor_instance_t *shoot_motor_2;
shoot_motor_instance_t *shoot_motor_3;

void Shoot_Init(void)
{
    shoot_motor_1 = Shoot_Motor_Init(&shoot_motor1_init);
    shoot_motor_2 = Shoot_Motor_Init(&shoot_motor2_init);
    shoot_motor_3 = Shoot_Motor_Init(&shoot_motor3_init);
}

void Shoot_Enable(void)
{
    Shoot_Motor_Enable(shoot_motor_1);
    Shoot_Motor_Enable(shoot_motor_2);
    Shoot_Motor_Enable(shoot_motor_3);
}

void Shoot_Stop(void)
{
    Shoot_Motor_Stop(shoot_motor_1);
    Shoot_Motor_Stop(shoot_motor_2);
    Shoot_Motor_Stop(shoot_motor_3);
}

void Shoot_SetAll(float speed)
{
    Shoot_Motor_SetTar(shoot_motor_1, speed);
    Shoot_Motor_SetTar(shoot_motor_2, speed);
    Shoot_Motor_SetTar(shoot_motor_3, -speed); // 这个是在下方的摩擦轮，转动方向与另外两个相反
}

void Shoot_Ready()
{
    /*采用或而不是与，避免电调偶发错误导致无法发弹*/
    if (shoot_motor_1->receive_flag == 0xA5 || shoot_motor_2->receive_flag == 0xA5 || shoot_motor_3->receive_flag == 0xA5)
    {
        fric_pretime_cnt++;
    }
    else
    fric_pretime_cnt = 0;

    if (fric_pretime_cnt > 3000) // 大于三秒允许发射
    {
        fire_ready_flag = 1;
    }
    else
        fire_ready_flag = 0;
}
/**
 * @brief 针对摩擦轮的射速转换函数
 * @param bullet_speed 目标弹速 (m/s)
 * @return float 电机转速参考值 (rpm)
 */
// static inline float BulletSpeed_to_RPM(float bullet_speed)
// {
//     // 防止输入过小的弹速值
//     if (bullet_speed < 0.5f)
//         return 0.0f;
//     // 1. 根据物理公式计算理论转速 (RPM)
//     // 公式推导: n = (v * 60) / (2 * PI * r * k)
//     float target_rpm = (bullet_speed * 60.0f) /
//                        (2.0f * PI * FRICTION_WHEEL_RADIUS * FRICTION_SLIP_COEFFICIENT);

//     // 确保计算出的转速不会超过电机的硬件极限
//     if (target_rpm > FRICTION_MOTOR_MAX_RPM)
//     {
//         target_rpm = FRICTION_MOTOR_MAX_RPM;
//     }
//     else if (target_rpm < 0.0f)
//     {
//         target_rpm = 0.0f; // 防止误输入负数
//     }

//     return target_rpm;
// }

void Get_Shoot_Mode(void)
{
    uint8_t pc_mode = vs_aim_packet_from_nuc.mode;

    switch (gimbal_mode)
    {
    case GIMBAL_MODE_SEMIAUTO:
        if (FRIC_flag)
        {
            shoot_mode = SHOOT_MODE_READY;
            if (rc_data->mouse.press_l)
            {
                shoot_mode = SHOOT_MODE_FIRE;
            }
            else if (rc_data->mouse.press_r)
            {

                if (pc_mode == PC_SHOOT_FIRE)
                {
                    shoot_mode = SHOOT_MODE_FIRE;
                }
                else if (pc_mode == PC_SHOOT_AIM)
                {
                    shoot_mode = SHOOT_MODE_READY;
                }
            }
        }
        else
            shoot_mode = SHOOT_MODE_STOP;
        break;

    case GIMBAL_MODE_AUTO:
        if (switch_is_up(rc_data->rc.switch_right))
        {
            if (pc_mode == PC_SHOOT_FIRE)
            {
                shoot_mode = SHOOT_MODE_FIRE;
            }
            else if (pc_mode == PC_SHOOT_AIM)
            {
                shoot_mode = SHOOT_MODE_READY;
            }
            else
            {
                shoot_mode = SHOOT_MODE_READY;
            }
        }
        else if (switch_is_mid(rc_data->rc.switch_right))
        {
            shoot_mode = SHOOT_MODE_READY;
        }
        else
        {
            shoot_mode = SHOOT_MODE_STOP;
        }
        break;

    case GIMBAL_MODE_MANUAL:
        if (switch_is_up(rc_data->rc.switch_right))
        {
            shoot_mode = SHOOT_MODE_FIRE;
        }
        else if (switch_is_mid(rc_data->rc.switch_right))
        {
            shoot_mode = SHOOT_MODE_READY;
        }
        else
        {
            shoot_mode = SHOOT_MODE_STOP;
        }
        break;

    case GIMBAL_MODE_STOP:
        shoot_mode = SHOOT_MODE_STOP;
        break;
    default:
        shoot_mode = SHOOT_MODE_STOP;
        break;
    }
}

void Shoot_State_Machine(void)
{
    static uint16_t init_count = 0;
    if (init_count < 1000)
    {
        init_count++;
        shoot_mode = SHOOT_MODE_STOP;
    }
    //Shoot_Ready();

    if (gimbal_mode)
    {
        switch (shoot_mode)
        {
        case SHOOT_MODE_FIRE:
//            if (fire_ready_flag == 1)
//            {
                uart2_tx_message.shoot_mode = 2;
//            }
//            else
//                uart2_tx_message.shoot_mode = 1;

            Shoot_Enable();
            Shoot_SetAll(FRICTION_MOTOR_RPM);
            break;

        case SHOOT_MODE_READY:
            uart2_tx_message.shoot_mode = 1;
            Shoot_Enable();
            Shoot_SetAll(FRICTION_MOTOR_RPM);
            break;

        case SHOOT_MODE_STOP:
            fric_pretime_cnt = 0;
            uart2_tx_message.shoot_mode = 0;
            Shoot_Stop();
            break;

        default:
            fric_pretime_cnt = 0;
            uart2_tx_message.shoot_mode = 0;
            Shoot_Stop();
            break;
        }
    }
    else
    {
        uart2_tx_message.shoot_mode = 0;
        Shoot_Stop();
    }
}
