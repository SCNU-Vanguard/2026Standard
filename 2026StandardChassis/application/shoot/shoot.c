/**
******************************************************************************
* @file    shoot.c
* @brief
* @author
******************************************************************************
*
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>

#include "shoot.h"
#include "chassis.h"
#include "rm_referee.h"
#include "referee_task.h"
#include "stdbool.h"

FeederState_e currentState = S_NORMAL;

/*拨弹盘物理参数*/
#define SHOOT_WHEEL_BULLET_COUNT 8.0f // 拨弹盘一圈的弹丸数
#define M2006_REDUCTION_RATIO 36.0f   // M2006电机减速比
/*堵转控制参数*/
#define SHOOT_CURRENT_STALL 8700.0f // 堵转电流
#define SHOOT_SPEED_STALL 2.0f      // 堵转速度
#define REVERSE_RADS -12.0f         // 反转速度
#define SHOOT_TIME_STALL 900        // 堵转时间
#define SHOOT_TIME_REVERSING 950     // 反转时间
#define SHOOT_TIME_STOP 1200        // 停转时间
/*热量控制参数*/
#define HEAT_SAFETY_MARGIN_HIGH 30.0f // 热量上限安全余量
#define HEAT_SAFETY_MARGIN_LOW 90.0f  // 热量下限安全余量

float shoot_hz = 5.0f; // 射击频率（发/秒）
uint16_t target_shoot_rads = 0;
uint16_t stall_cnt = 0;          // 堵转时间计数
uint16_t reverse_cnt = 0;        // 反转时间计数
uint16_t stop_cnt = 0;           // 停止计数
static bool heat_locked = false; // 热量锁定标志，初始为false,表示未锁定


PID_t chassis_2006_speed_pid = {
    .kp = 80.0f,
    .ki = 15.0f,
    .kd = 0.0f,
    .output_limit = 9000.0f,
    .integral_limit = 9000.0f,
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

/**
 * @brief 将目标射击频率(Hz)转换为电机转子的目标角速度(rad/s)
 * @param hz 目标射频（每秒发数）
 * @return float 电机转子的目标角速度 (单位: rad/s)，用于传给 DJI_Motor_Set_Ref
 */
static inline float BulletFreq_to_RadS(float hz) // inline 直接调用内容，提升效率
{
    // 推导过程：
    // 1. 拨弹轴每秒转数 (rps) = 目标频率 / 一圈弹丸数
    // 2. 电机转子每秒转数 (rps) = 拨弹轴rps * 减速比
    // 3. 电机转子角速度 (rad/s) = 电机转子rps * 2 * PI

    float target_motor_rad_s = (hz / SHOOT_WHEEL_BULLET_COUNT) * M2006_REDUCTION_RATIO * (2.0f * PI);

    return target_motor_rad_s;
}

// void Update_Bullet_Count(DJI_motor_instance_t *motor)
// {
//   const float ANGLE_PER_BULLET = 1620.0f; // 电机转子需转过的角度

//    // 计算自上次计数以来，电机转子又转了多少度
//    float delta_angle = motor->measure.total_angle - motor->last_shot_angle;

//    // 如果增量超过了 1620 度（一颗子弹的位移）
//    if (delta_angle >= ANGLE_PER_BULLET)
//    {
//        // 计算本次增加了几颗子弹（防止单次调用跨度过大）
//        uint32_t new_bullets = (uint32_t)(delta_angle / ANGLE_PER_BULLET);

//        motor->total_bullets += new_bullets;

//        // 更新记录点：按子弹步长增加，保留不足一发的余数角度
//        motor->last_shot_angle += new_bullets * ANGLE_PER_BULLET;
//    }
//    // 如果发生反转（如手动回拨），delta_angle 可能为负，根据需求决定是否扣除子弹
//    else if (delta_angle <= -ANGLE_PER_BULLET)
//    {
//        uint32_t lost_bullet = (uint32_t)(-delta_angle / ANGLE_PER_BULLET);
//        motor->total_bullets = (motor->total_bullets > lost_bullet) ? (motor->total_bullets - lost_bullet) : 0;
//        motor->last_shot_angle -= lost_bullet * ANGLE_PER_BULLET;
//    }
// }

void Update_OverHeated(void)
{
    float heat_now = referee_outer_info->PowerHeatData.shooter_17mm_barrel_heat;   // 当前热量
    float heat_limit = referee_outer_info->RobotPerformance.shooter_barrel_heat_limit; // 热量上限

    if (heat_now > heat_limit - HEAT_SAFETY_MARGIN_HIGH)
    {
        heat_locked = true; // 锁定射击
    }
    else if (heat_now < heat_limit - HEAT_SAFETY_MARGIN_LOW) // 当热量降到安全余量以下时解除锁定
    {
        heat_locked = false; // 解锁射击
    }
}

float Stall_Control_Loop(float target_rads)
{
    float shoot_speed = chassis_shoot_motor->measure.speed;
    float shoot_current = chassis_shoot_motor->measure.real_current;
    // bool reverse_first = 1;
    if (currentState == S_NORMAL)
    {
        // 1. 堵转检测
        if (fabsf(shoot_speed) < SHOOT_SPEED_STALL && fabsf(shoot_current) > SHOOT_CURRENT_STALL)
        {
            stall_cnt++;
            if (stall_cnt >= SHOOT_TIME_STALL)
            {
                // 确认堵转，切换到反转状态
                currentState = S_REVERSING;
                stall_cnt = 0;
                reverse_cnt = 0;
                stop_cnt = 0;
                PID_Clear(chassis_shoot_motor->motor_controller.speed_PID);
            }
        }
        else
        {
            // 正常状态，堵转计数器清零
            stall_cnt = 0;
        }
    }
    else if (currentState == S_REVERSING)
    {
        if (stop_cnt <= SHOOT_TIME_STOP)
        {
            // 保持停止，让拨弹盘自然泄力回转
            Shoot_Stop();
            stop_cnt++;
            return 0.0f; // 停止状态，目标转速为0
        } 
        else
            reverse_cnt++;

        if (reverse_cnt >= SHOOT_TIME_REVERSING)
        {
            currentState = S_NORMAL;

            // 调用PID清除积分函数，防止恢复正转时猛冲
            PID_Clear(chassis_shoot_motor->motor_controller.speed_PID);
            return target_rads;
        }
        return REVERSE_RADS; // 处于反转状态，强制覆盖目标转速
    }

    return target_rads; // 正常状态，原样返回目标转速
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
 
    // Update_OverHeated(); // 更新过热状态
    // if (heat_locked == true && shoot_mode == SHOOT_MODE_FIRE)
    // {
    //     shoot_mode = SHOOT_MODE_READY;
    // }

//    if(gimbal_motor_yaw ->receive_data.position > 1.88f || gimbal_motor_yaw ->receive_data.position < 1.39f)
//    {
//        shoot_mode = SHOOT_MODE_READY;
//    }


    //  if(gimbal_angle_yaw_motor2imu > 0.25f || gimbal_angle_yaw_motor2imu < -2.5f)
    // {
    //     shoot_mode = SHOOT_MODE_READY;
    // }
    if (chassis_mode)
    {
        switch (shoot_mode)
        {

        case SHOOT_MODE_FIRE:
            Shoot_Enable();
            target_shoot_rads = BulletFreq_to_RadS(shoot_hz);
            float final_target_rads = Stall_Control_Loop(target_shoot_rads);
            DJI_Motor_Set_Ref(chassis_shoot_motor, final_target_rads);
            break;
        case SHOOT_MODE_READY:
            Shoot_Stop();
            break;
        case SHOOT_MODE_STOP:
            Shoot_Stop();
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
