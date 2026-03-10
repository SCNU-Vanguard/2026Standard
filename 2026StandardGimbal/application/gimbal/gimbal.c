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
#include <math.h>

#define TARGET_STEP_PITCH_AUTO 0.0003f
#define TARGET_STEP_PITCH_MANUAL 0.0015f
#define PITCH_DOWN_LIMIT 0.30f
#define PITCH_UP_LIMIT -0.50f
#define RC_SENSITIVITY_PITCH 0.0000025f
#define RC_SENSITIVITY_YAW 0.00002f
// 底盘速度相关参数
#define RC_SENSITIVITY_X 0.005f // 遥控器X轴速度灵敏度
#define RC_SENSITIVITY_Y 0.005f // 遥控器Y轴速度灵敏度

#include "gimbal_task.h"
#include "gimbal.h"

#include "message_center.h"

#include "DJI_motor.h"
#include "DM_motor.h"
#include "remote_control.h"
#include "INS.h"
#include "Serial.h"
#include "VPC.h"

// 云台相关参数
float angle_pitch_offset = 5.22f;
float yaw_zero_offset = 0.0f;
uint8_t effective_mode = 0;
uint8_t gimbal_mode = 0;
uint8_t gimbal_mode_last = 0;
float target_angle_pitch = 0;
float target_angle_yaw = 0;
float target_angle_pitch_temp = 0;
float angle_pitch = 0;
float angle_pitch_motor = 0;
float angle_yaw = 0;
float angle_pitch_motor2imu = 0;
float angle_yaw_motor2imu = 0;

float pitch_speed_feedback = 0;
float pitch_angle_feedback = 0;
float torque_speed_feedforward = 0;
// 底盘速度相关参数
static const float chassis_gear_speed[SPEED_GEAR_COUNT] = {2.0f, 3.0f, 4.0f}; // 不同档位（低，中，高）对应的底盘最大速度
Speed_Gear_e current_gear = GEAR_PRECISION;                                   // 默认低速档

PID_t gimbal_6020_angle_pid = {
    // TODO(GUATAI:)
    // .kp = 50.0f,
    // .ki = 1.0f,
    // .kd = 0.0f,
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 3.0f,
    .output_limit = 500.0f,
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
};

PID_t gimbal_6020_speed_pid = {
    // TODO(GUATAI:)
    // .kp = 50.0f,
    // .ki = 1.0f,
    // .kd = 0.0f,
    .kp = 6000.0f,
    .ki = 15.0f,
    .kd = 1.5f,
    .output_limit = 15000.0f,
    .integral_limit = 10000.0f,
    .dead_band = 0.0f,
};

motor_init_config_t gimbal_6020_init = {
    .controller_param_init_config = {
        .angle_PID = &gimbal_6020_angle_pid,
        .speed_PID = &gimbal_6020_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = &pitch_angle_feedback, // NULL,
        .other_speed_feedback_ptr = &pitch_speed_feedback, // NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = &torque_speed_feedforward, // NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_AND_SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,

        .angle_feedback_source = OTHER_FEED, // MOTOR_FEED,
        .speed_feedback_source = OTHER_FEED,

        .feedforward_flag = TORQUE_FEEDFORWARD,

        .control_button = TORQUE_DIRECT_CONTROL, // POLYCYCLIC_LOOP_CONTROL, // TORQUE_DIRECT_CONTROL, //扭矩直接开环控制
    },

    .motor_type = GM6020,

    .control_6020_flag = ELECTRICITY,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x01,
        .rx_id = 0x01,
    },
};

DJI_motor_instance_t *gimbal_motor_pitch;

digital_PID_t gimbal_pitch_angle_digital_pid;
digital_PID_t gimbal_pitch_speed_digital_pid;

#include "user_lib.h"
ramp_init_config_t gimbal_pitch_speed_ramp = {
    .increase_value = 0.5f,
    .decrease_value = 0.5f,
    .frame_period = 0.001f,
    .max_value = 0.0f,
    .min_value = 0.0f};

ramp_function_source_t *gimbal_pitch_speed_ramp_source;

void Gimbal_Init(void)
{
    digital_PID_t gimbal_temp_pid = {
        .Kp = 40.0f, //12.5f
        .Ki = 0.0f,
        .Kd = 5.0f,
        .Kf = 50.0f,

        .dead_band = 0.0f,

        .output_LPF_RC = 0.0f,

        .improve = PID_PROPORTIONAL_ON_MEASUREMENT,

        .integral_limit = 0.0f,
        .forward_limit = 50.0f,

        .output_max = 100.0f,
    };

    gimbal_pitch_angle_digital_pid = gimbal_temp_pid;

    gimbal_temp_pid.Kp = 1.0f;//2.0f;
    gimbal_temp_pid.Ki = 0.0075f;
    gimbal_temp_pid.Kd = 15.0f;
    gimbal_temp_pid.Kf = 0.05f;
    gimbal_temp_pid.output_LPF_RC = 0.0f;
    gimbal_temp_pid.improve = PID_PROPORTIONAL_ON_MEASUREMENT;
    gimbal_temp_pid.integral_limit = 0.0f;
    gimbal_temp_pid.forward_limit = 0.75f;
    gimbal_temp_pid.output_max = 2.2f;

    gimbal_pitch_speed_digital_pid = gimbal_temp_pid;

    gimbal_pitch_speed_ramp_source = ramp_init(&gimbal_pitch_speed_ramp);

    //    gimbal_6020_init->control_6020_flag = ELECTRICITY; // 设置6020电机控制标志
    gimbal_motor_pitch = DJI_Motor_Init(&gimbal_6020_init);
}

/*根据遥控器输入值判断云台模式*/
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

/**
 * @brief 根据云台模式获取底盘模式
 * @param   mode 云台模式
 * @retval  chassis_mode 底盘模式
 */
uint8_t Get_Chassis_Mode(uint8_t mode)
{
    uint8_t chassis_mode = CHASSIS_MODE_STOP;
    switch (mode)
    {
    case GIMBAL_MODE_AUTO:
        chassis_mode = CHASSIS_MODE_AUTO;
        break;

    case GIMBAL_MODE_MANUAL:
        chassis_mode = CHASSIS_MODE_MANUAL;
        break;

    case GIMBAL_MODE_STOP:
        chassis_mode = CHASSIS_MODE_STOP;
        break;
    default:
        chassis_mode = CHASSIS_MODE_STOP;
        break;
    }
    return chassis_mode;
}

/**
 * @brief 根据云台模式获取底盘X方向速度
 * @param   mode 云台模式
 * @retval  chassis_x_speed 底盘X方向速度
 */
float Get_Chassis_X_Speed(uint8_t mode)
{
    float x_speed = 0;
    float manual_chassis_x_speed = 0;

    Update_Chassis_Gear(); // shift更新底盘档位
    // 引入键鼠判断，优先使用键鼠输入的速度
    if (rc_data->key[KEY_PRESS].w)
    {
        float manual_chassis_x_speed = chassis_gear_speed[current_gear]; // 键盘前进
    }
    else if (rc_data->key[KEY_PRESS].s)
    {
        float manual_chassis_x_speed = -chassis_gear_speed[current_gear]; // 键盘后退
    }
    else
    {
        manual_chassis_x_speed = rc_data->rc.rocker_l1 * RC_SENSITIVITY_X; // 遥控器控制
    }
    switch (mode)
    {
    case GIMBAL_MODE_AUTO:
        x_speed = nv_aim_packet_from_nuc.vx;
        break;
    case GIMBAL_MODE_MANUAL:
        x_speed = manual_chassis_x_speed;
        break;
    case GIMBAL_MODE_STOP:
        x_speed = manual_chassis_x_speed;
        break;
    default:
        x_speed = manual_chassis_x_speed;
        break;
    }
    return x_speed;
}

/**
 * @brief 根据云台模式获取底盘Y方向速度
 * @param   mode 云台模式
 * @retval  chassis_y_speed 底盘Y方向速度
 */
float Get_Chassis_Y_Speed(uint8_t mode)
{
    float y_speed = 0;
    float manual_chassis_y_speed = 0;

    // 引入键鼠判断，优先使用键鼠输入的速度
    if (rc_data->key[KEY_PRESS].a)
    {
        float manual_chassis_y_speed = -chassis_gear_speed[current_gear]; // 键盘左移
    }
    else if (rc_data->key[KEY_PRESS].d)
    {
        float manual_chassis_y_speed = chassis_gear_speed[current_gear]; // 键盘右移
    }
    else
    {
        manual_chassis_y_speed = rc_data->rc.rocker_l_ * RC_SENSITIVITY_Y; // 遥控器控制
    }
    switch (mode)
    {
    case GIMBAL_MODE_AUTO:
        y_speed = nv_aim_packet_from_nuc.vy;
        break;
    case GIMBAL_MODE_MANUAL:
        y_speed = manual_chassis_y_speed;
        break;
    case GIMBAL_MODE_STOP:
        y_speed = manual_chassis_y_speed;
        break;
    default:
        y_speed = manual_chassis_y_speed;
        break;
    }
    return y_speed;
}

float Get_Target_Angle_Yaw(uint8_t mode)
{
    switch (mode)
    {
    case GIMBAL_MODE_AUTO:
        return target_angle_yaw; //-(target_angle_yaw - angle_yaw);
    case GIMBAL_MODE_MANUAL:
        return -(target_angle_yaw - angle_yaw);
    case GIMBAL_MODE_STOP:
        return -(target_angle_yaw - angle_yaw);
    default:
        return -(target_angle_yaw - angle_yaw);
    }
}

void Chassis_Control(void)
{
    uart2_tx_message.chassis_mode = Get_Chassis_Mode(gimbal_mode); //(gimbal_mode == GIMBAL_MODE_MANUAL);
    uart2_tx_message.delta_target_angle_yaw = Get_Target_Angle_Yaw(gimbal_mode);
    // aaa = Get_Target_Angle_Yaw(gimbal_mode);
    uart2_tx_message.target_x_speed = Get_Chassis_X_Speed(gimbal_mode); // rc_data->rc.rocker_l1 * 0.005f;
    uart2_tx_message.target_y_speed = Get_Chassis_Y_Speed(gimbal_mode); // rc_data->rc.rocker_l_ * 0.005f;
    uart2_tx_message.INS_yaw = INS.Yaw;
    uart2_tx_message.INS_Gyro_Z = INS.Gyro[IMU_Z];
    if (rc_data->rc.dial)
    {
        uart2_tx_message.target_omega_speed = 2.0f * PI;
    }
    else
    {
        uart2_tx_message.target_omega_speed = 0.0f;
    }
}

float Delta_Target_Angle_Control(float pitch_step)
{
    if (fabs(target_angle_pitch - target_angle_pitch_temp) >= pitch_step)
    {
        return (target_angle_pitch - target_angle_pitch_temp) >= 0
                   ? (target_angle_pitch_temp + pitch_step)
                   : (target_angle_pitch_temp - pitch_step);
    }
    else
    {
        return target_angle_pitch;
    }
}

float k_forward = 1.0f;

float temp_speed_target = 0.0f;

float t_p = 0.0f;
/*根据云台状态机进行相应的操作*/
void Gimbal_State_Machine(void)
{
    static uint16_t init_count = 0;

    // TODO(GUATAI) 電流
    // torque_speed_feedforward = -k_forward * (0.1f * 0.65f * 9.8f * 16384.0f / 2.23f * cosf(INS.Pitch)); // 0.1f是负载质量，0.8f是效率估计，2.23f是6020电机转速常数，16384.0f是编码器分辨率的一半
    // 力矩
    torque_speed_feedforward = -k_forward * (0.1f * 0.65f * 9.8f * cosf(INS.Pitch));

    angle_pitch_motor2imu = -gimbal_motor_pitch->measure.rad + angle_pitch_offset;
    angle_yaw_motor2imu = uart2_rx_message.gimbal_angle_yaw_motor2imu;
    angle_pitch = INS.Pitch;
    angle_yaw = INS.Yaw;

    // MotorToQuaternion(&motor_q, angle_yaw_motor2imu, angle_pitch_motor2imu);
    pitch_speed_feedback = INS.Gyro[IMU_X]; // 将imu的原始数据作为pitch电机的速度反馈
    pitch_angle_feedback = INS.Pitch;       // 将imu的角度作为pitch电机的角度反馈

    if (init_count < 1000)
    {
        init_count++;
        gimbal_mode = GIMBAL_MODE_STOP;
    }

    if ((gimbal_mode == GIMBAL_MODE_AUTO && gimbal_mode_last != GIMBAL_MODE_AUTO) || (gimbal_mode == GIMBAL_MODE_MANUAL && gimbal_mode_last != GIMBAL_MODE_MANUAL))
    {
        // 只有在切换的这一秒，让目标等于当前，实现平滑启动,同时清零pid积分项，防止突变
        target_angle_yaw = angle_yaw;
        target_angle_pitch = angle_pitch;
        Digital_PID_Clear(&gimbal_pitch_angle_digital_pid);
        Digital_PID_Clear(&gimbal_pitch_speed_digital_pid);
    }

    switch (gimbal_mode)
    {
    case GIMBAL_MODE_AUTO:
        // imu控制
        if (xSemaphoreTake(g_xSemVPC, 0) == pdPASS)
        {
            if (vs_aim_packet_from_nuc.pitch != 0.0f)
            {
                // 安全限幅，控制转动角度始终不超过机械限制 //-0.55 0.44
                target_angle_pitch = vs_aim_packet_from_nuc.pitch;
                target_angle_pitch = Value_Limit(target_angle_pitch, PITCH_UP_LIMIT, PITCH_DOWN_LIMIT);
                // target_angle_pitch_temp = Delta_Target_Angle_Control(0.03f);
                // target_angle_pitch = target_angle_pitch_temp;
                // 步进平滑器
                // target_angle_pitch_temp = Delta_Target_Angle_Control(TARGET_STEP_PITCH_AUTO);

                // DJI_Motor_Set_Ref(gimbal_motor_pitch, target_angle_pitch_temp);
            }
            if (vs_aim_packet_from_nuc.yaw != 0.0f)
            {
                target_angle_yaw = vs_aim_packet_from_nuc.yaw;
            }
        }
        DJI_Motor_Enable(gimbal_motor_pitch);
                // 外置PID控制
        Digital_PID_Position(&gimbal_pitch_angle_digital_pid, INS.Pitch, target_angle_pitch);
        // gimbal_pitch_speed_ramp_source->real_value = ramp_calc(gimbal_pitch_speed_ramp_source, gimbal_pitch_angle_digital_pid.output);
        // temp_speed_target = gimbal_pitch_speed_ramp_source->real_value;
        //Digital_PID_Increment(&gimbal_pitch_speed_digital_pid, INS.Gyro[IMU_X], temp_speed_target);
        Digital_PID_Increment(&gimbal_pitch_speed_digital_pid, INS.Gyro[IMU_X], gimbal_pitch_angle_digital_pid.output);
        DJI_Motor_Set_Ref(gimbal_motor_pitch, -gimbal_pitch_speed_digital_pid.output);
        gimbal_mode_last = GIMBAL_MODE_AUTO;
        break;

    case GIMBAL_MODE_MANUAL:
        target_angle_pitch = target_angle_pitch - rc_data->rc.rocker_r1 * 0.0000025f;
        target_angle_pitch = Value_Limit(target_angle_pitch, PITCH_UP_LIMIT, PITCH_DOWN_LIMIT);
        target_angle_pitch_temp = Delta_Target_Angle_Control(1.0f); // 0.0015f// 0.0005f
        target_angle_pitch = target_angle_pitch_temp;
        // target_angle_pitch = target_angle_pitch_temp;

        target_angle_yaw = angle_yaw + rc_data->rc.rocker_r_ * 0.00002f;

        // DJI_Motor_Set_Ref(gimbal_motor_pitch, target_angle_pitch_temp);
        DJI_Motor_Enable(gimbal_motor_pitch);
        // 外置PID控制
        Digital_PID_Position(&gimbal_pitch_angle_digital_pid, INS.Pitch, target_angle_pitch);
        // gimbal_pitch_speed_ramp_source->real_value = ramp_calc(gimbal_pitch_speed_ramp_source, gimbal_pitch_angle_digital_pid.output);
        // temp_speed_target = gimbal_pitch_speed_ramp_source->real_value;
        //Digital_PID_Increment(&gimbal_pitch_speed_digital_pid, INS.Gyro[IMU_X], temp_speed_target);
        Digital_PID_Increment(&gimbal_pitch_speed_digital_pid, INS.Gyro[IMU_X], gimbal_pitch_angle_digital_pid.output);
        DJI_Motor_Set_Ref(gimbal_motor_pitch, -gimbal_pitch_speed_digital_pid.output);
        gimbal_mode_last = GIMBAL_MODE_MANUAL;
        break;

    case GIMBAL_MODE_STOP:
        target_angle_pitch = angle_pitch;
        target_angle_pitch_temp = angle_pitch;
        target_angle_yaw = angle_yaw;
        Digital_PID_Clear(&gimbal_pitch_angle_digital_pid);
        Digital_PID_Clear(&gimbal_pitch_speed_digital_pid);
        DJI_Motor_Stop(gimbal_motor_pitch);
        gimbal_mode_last = GIMBAL_MODE_STOP;
        break;

    default:
        target_angle_pitch = angle_pitch;
        target_angle_pitch_temp = angle_pitch;
        target_angle_yaw = angle_yaw;
        DJI_Motor_Stop(gimbal_motor_pitch);
        gimbal_mode_last = GIMBAL_MODE_STOP;
        break;
    }

    DJI_Motor_Control();
}

void Remote_Deadzone_Control(void)
{
    if (abs(rc_data->rc.rocker_r_) <= 20)
    {
        rc_data->rc.rocker_r_ = 0;
    }
    if (abs(rc_data->rc.rocker_r1) <= 20)
    {
        rc_data->rc.rocker_r1 = 0;
    }
    if (abs(rc_data->rc.rocker_l_) <= 20)
    {
        rc_data->rc.rocker_l_ = 0;
    }
    if (abs(rc_data->rc.rocker_l1) <= 20)
    {
        rc_data->rc.rocker_l1 = 0;
    }
    if (abs(rc_data->rc.dial) <= 659)
    {
        rc_data->rc.dial = 0;
    }
}

// void Auto_Deadzone_Control(void)
// {
//     if (fabsf(vs_aim_packet_from_nuc.input_data.shoot_pitch) <= 0.02f)
//     {
//         vs_aim_packet_from_nuc.input_data.shoot_pitch = 0.0f;
//     }
//     if (fabsf(vs_aim_packet_from_nuc.input_data.shoot_yaw) <= 0.02f)
//     {
//         vs_aim_packet_from_nuc.input_data.shoot_yaw = 0.0f;
//     }
// }