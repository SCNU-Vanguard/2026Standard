/**
 * @file    chassis.c
 * @brief   舵轮底盘控制模块
 * @author  RoboMaster Team
 */

// 底盘布局 (俯视图)
// 4-------电池-------1
// |                  |
// |                  |
// |                  |
// |                  |
// |                  |
// |                  |
// 3------------------2

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "chassis.h"
#include "chassis_task.h"

#include "DM_motor.h"
#include "DJI_motor.h"
#include "rs485.h"

// 舵向电机零点偏移 (弧度)
#define CHASSIS_MOTOR_DIRECT_ZEROPOINT_1 620.0f / 8192.0f * 2 * PI
#define CHASSIS_MOTOR_DIRECT_ZEROPOINT_2 1270.0f / 8192.0f * 2 * PI
#define CHASSIS_MOTOR_DIRECT_ZEROPOINT_3 3380.0f / 8192.0f * 2 * PI
#define CHASSIS_MOTOR_DIRECT_ZEROPOINT_4 680.0f / 8192.0f * 2 * PI

// 底盘物理参数
#define CHASSIS_RADIUS 0.21917f      // 底盘半径 (m)
#define WHEEL_RADIUS 0.055f          // 轮子半径 (m)
#define WHEEL_REDUCTION_RATIO 14.88f // 轮子减速比

// 速度步长
#define ACC_SPEED_STEP 0.01f // 加速步长
#define DEC_SPEED_STEP 0.01f // 减速步长

float target_angle_yaw = 0;
float target_angle_yaw_temp = 0;
uint8_t chassis_mode = 0;
float chassis_mode_last = 0;
uint16_t init_count = 0;
float chassis_actual_omega = 0;
float gimbal_angle_yaw_motor2imu = 0.0f;
float gimbal_angle_yaw_motor = 0.0f;
float gimbal_angle_yaw_motor_last = 0.0f;
float yaw_angle_feedback = 0.0f;
float yaw_speed_feedback = 0.0f;
float yaw_torque_val = 0.0f;

// int brake_timer = 0;
Speed_Ramp_t chassis_x_speed_ramp = {
    .last_speed = 0.0f,
    .accel_step = ACC_SPEED_STEP,
    .decel_step = DEC_SPEED_STEP,
};

Speed_Ramp_t chassis_y_speed_ramp = {
    .last_speed = 0.0f,
    .accel_step = ACC_SPEED_STEP,
    .decel_step = DEC_SPEED_STEP,
};
PID_t chassis_3508_speed_pid = {
    .kp = 12.0f,  // 10.0f
    .ki = 0.001f, // 0.01f
    .kd = 0.0f,
    .output_limit = 10000.0f,
    .integral_limit = 10000.0f,
    .dead_band = 0.0f,
};

PID_t chassis_6020_angle_pid = {
    .kp = 100.0f,
    .ki = 0.0f,
    .kd = 400.0f,
    .output_limit = 20.0f,
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
};

PID_t chassis_6020_speed_pid = {
    .kp = 400.0f,
    .ki = 20.0f,
    .kd = 0.0f,
    .output_limit = 15000.0f,
    .integral_limit = 15000.0f,
    .dead_band = 0.0f,
};

PID_t gimbal_4310_angle_pid = {
    .kp = 12.0f,//15.0f,//12.0f 
    .ki = 0.0f, //0.0f,
    .kd = 3.0f,//1.2f, //3.0f,  
    .output_limit = 20.0f,//5.0f,
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
};

PID_t gimbal_4310_speed_pid = {
    .kp = 1.5f, //1.5f,
    .ki = 0.01f,//0.005f,//0.01f,
    .kd = 0.5f,//0.005f,//5.0f,
    .kf = 25.0f,
    .output_limit = 15.0f,
    .integral_limit = 10.0f,
    .fout_limit = 5.0f,
    .dead_band = 0.0f,
};

motor_init_config_t chassis_3508_init_1 = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_3508_speed_pid,
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

    .motor_type = M3508,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x01,
        .rx_id = 0x01,
    },

    .motor_control_type = TORQUE_LOOP_CONTRO,
};

motor_init_config_t chassis_3508_init_2 = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_3508_speed_pid,
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

    .motor_type = M3508,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x02,
        .rx_id = 0x02,
    },

    .motor_control_type = TORQUE_LOOP_CONTRO,
};

motor_init_config_t chassis_3508_init_3 = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_3508_speed_pid,
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

    .motor_type = M3508,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x03,
        .rx_id = 0x03,
    },

    .motor_control_type = TORQUE_LOOP_CONTRO,
};

motor_init_config_t chassis_3508_init_4 = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_3508_speed_pid,
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

    .motor_type = M3508,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x04,
        .rx_id = 0x04,
    },

    .motor_control_type = TORQUE_LOOP_CONTRO,
};

motor_init_config_t chassis_6020_init_1 = {
    .controller_param_init_config = {
        .angle_PID = &chassis_6020_angle_pid,
        .speed_PID = &chassis_6020_speed_pid,
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
        .can_handle = &hfdcan2,
        .tx_id = 0x01,
        .rx_id = 0x01,
    },
};

motor_init_config_t chassis_6020_init_2 = {
    .controller_param_init_config = {
        .angle_PID = &chassis_6020_angle_pid,
        .speed_PID = &chassis_6020_speed_pid,
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
        .can_handle = &hfdcan2,
        .tx_id = 0x02,
        .rx_id = 0x02,
    },
};

motor_init_config_t chassis_6020_init_3 = {
    .controller_param_init_config = {
        .angle_PID = &chassis_6020_angle_pid,
        .speed_PID = &chassis_6020_speed_pid,
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
        .can_handle = &hfdcan2,
        .tx_id = 0x03,
        .rx_id = 0x03,
    },
};

motor_init_config_t chassis_6020_init_4 = {
    .controller_param_init_config = {
        .angle_PID = &chassis_6020_angle_pid,
        .speed_PID = &chassis_6020_speed_pid,
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
        .can_handle = &hfdcan2,
        .tx_id = 0x04,
        .rx_id = 0x04,
    },
};

motor_init_config_t gimbal_4310_init = {
    .controller_param_init_config = {
        .angle_PID = &gimbal_4310_angle_pid,
        .speed_PID = &gimbal_4310_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = &yaw_angle_feedback,
        .other_speed_feedback_ptr = &yaw_speed_feedback,//NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL, //&yaw_torque_val, // 扭矩前馈输入值的地址
        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_AND_SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = OTHER_FEED,
        .speed_feedback_source = OTHER_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = DM4310,

    .can_init_config = {
        .can_handle = &hfdcan3,
        .tx_id = 0x01,
        .rx_id = 0x11,
    },
};

/*轮向电机*/
DJI_motor_instance_t *chassis_motor_drive_1;
DJI_motor_instance_t *chassis_motor_drive_2;
DJI_motor_instance_t *chassis_motor_drive_3;
DJI_motor_instance_t *chassis_motor_drive_4;
/*舵向电机*/
DJI_motor_instance_t *chassis_motor_direct_1;
DJI_motor_instance_t *chassis_motor_direct_2;
DJI_motor_instance_t *chassis_motor_direct_3;
DJI_motor_instance_t *chassis_motor_direct_4;
/*云台yaw轴电机*/
DM_motor_t *gimbal_motor_yaw; // 在初始化中默认使用MIT模式

void Chassis_Init(void)
{
    chassis_motor_drive_1 = DJI_Motor_Init(&chassis_3508_init_1);
    chassis_motor_drive_2 = DJI_Motor_Init(&chassis_3508_init_2);
    chassis_motor_drive_3 = DJI_Motor_Init(&chassis_3508_init_3);
    chassis_motor_drive_4 = DJI_Motor_Init(&chassis_3508_init_4);

    chassis_motor_direct_1 = DJI_Motor_Init(&chassis_6020_init_1);
    chassis_motor_direct_2 = DJI_Motor_Init(&chassis_6020_init_2);
    chassis_motor_direct_3 = DJI_Motor_Init(&chassis_6020_init_3);
    chassis_motor_direct_4 = DJI_Motor_Init(&chassis_6020_init_4);

    gimbal_motor_yaw = DM_Motor_Init(&gimbal_4310_init);
}

void Chassis_Enable(void)
{
    DJI_Motor_Enable(chassis_motor_drive_1);
    DJI_Motor_Enable(chassis_motor_drive_2);
    DJI_Motor_Enable(chassis_motor_drive_3);
    DJI_Motor_Enable(chassis_motor_drive_4);

    DJI_Motor_Enable(chassis_motor_direct_1);
    DJI_Motor_Enable(chassis_motor_direct_2);
    DJI_Motor_Enable(chassis_motor_direct_3);
    DJI_Motor_Enable(chassis_motor_direct_4);

    DM_Motor_Start(gimbal_motor_yaw);
    DM_Motor_ENABLE(gimbal_motor_yaw);
}

void Chassis_Stop(void)
{
    DJI_Motor_Stop(chassis_motor_drive_1);
    DJI_Motor_Stop(chassis_motor_drive_2);
    DJI_Motor_Stop(chassis_motor_drive_3);
    DJI_Motor_Stop(chassis_motor_drive_4);

    DJI_Motor_Stop(chassis_motor_direct_1);
    DJI_Motor_Stop(chassis_motor_direct_2);
    DJI_Motor_Stop(chassis_motor_direct_3);
    DJI_Motor_Stop(chassis_motor_direct_4);

    DM_Motor_Stop(gimbal_motor_yaw);
    DM_Motor_DISABLE(gimbal_motor_yaw);
}

/*底盘解算*/
void Chassis_Resolving(float x_speed, float y_speed, float omega_speed, float gimbal_angle_yaw_offset)
{
    // 舵向电机零点偏移
    float chassis_motor_direct_zeropoint[4] = {CHASSIS_MOTOR_DIRECT_ZEROPOINT_1,
                                               CHASSIS_MOTOR_DIRECT_ZEROPOINT_2,
                                               CHASSIS_MOTOR_DIRECT_ZEROPOINT_3,
                                               CHASSIS_MOTOR_DIRECT_ZEROPOINT_4};
    // 电机实例数组
    DJI_motor_instance_t *chassis_motor_direct[4] = {chassis_motor_direct_1,
                                                     chassis_motor_direct_2,
                                                     chassis_motor_direct_3,
                                                     chassis_motor_direct_4};
    DJI_motor_instance_t *chassis_motor_drive[4] = {chassis_motor_drive_1,
                                                    chassis_motor_drive_2,
                                                    chassis_motor_drive_3,
                                                    chassis_motor_drive_4};

    // 轮子位置角度：右前45°，右后-45°，左后-135°，左前135°
    float wheel_position_angle[4] = {PI / 4, -PI / 4, -3 * PI / 4, 3 * PI / 4};

    // 坐标系转换：云台坐标系 → 底盘坐标系
    float cos_gimbal = cosf(gimbal_angle_yaw_offset);
    float sin_gimbal = sinf(gimbal_angle_yaw_offset);
    float chassis_x_speed = x_speed * cos_gimbal + y_speed * sin_gimbal;
    float chassis_y_speed = -x_speed * sin_gimbal + y_speed * cos_gimbal;
    float target_angle = 0;
    float target_speed = 0;

    // 舵轮解算主循环
    for (int i = 0; i < 4; i++)
    {
        if (chassis_x_speed || chassis_y_speed || omega_speed)
        {
            // 自旋方向系数：对1、3号轮取反，使所有轮子同向旋转
            float spin_direction = (i == 0 || i == 2) ? -1.0f : 1.0f;

            // 合成速度：平移 + 自旋切向分量
            float vx_total = chassis_x_speed - spin_direction * omega_speed * CHASSIS_RADIUS * sinf(wheel_position_angle[i]);
            float vy_total = chassis_y_speed + spin_direction * omega_speed * CHASSIS_RADIUS * cosf(wheel_position_angle[i]);

            // 目标方向和速度
            float target_angle_rad = atan2f(vy_total, vx_total);
            float speed_magnitude = sqrtf(vx_total * vx_total + vy_total * vy_total);

            // 当前舵向角度
            float current_angle_rad = chassis_motor_direct[i]->measure.rad - chassis_motor_direct_zeropoint[i];

            // 劣弧优化：计算最短路径角度差
            float angle_diff = target_angle_rad - current_angle_rad;
            while (angle_diff > PI)
                angle_diff -= 2 * PI;
            while (angle_diff < -PI)
                angle_diff += 2 * PI;

            // 90度优化：大角度时反转轮子
            if (angle_diff > PI / 2)
            {
                angle_diff -= PI;
                speed_magnitude = -speed_magnitude;
            }
            else if (angle_diff < -PI / 2)
            {
                angle_diff += PI;
                speed_magnitude = -speed_magnitude;
            }

            // 目标角度和转速
            // 速度转换：m/s → rad/s
            // target_speed = (v / r) × 减速比
            target_angle = chassis_motor_direct[i]->measure.rad + angle_diff;
            target_speed = speed_magnitude / WHEEL_RADIUS * WHEEL_REDUCTION_RATIO;

            // 发送控制指令
            DJI_Motor_Set_Ref(chassis_motor_direct[i], target_angle);
            DJI_Motor_Set_Ref(chassis_motor_drive[i], target_speed);
        }
        else
        {
            DJI_Motor_Set_Ref(chassis_motor_drive[i], 0);
        }
    }
}

/**
 * @brief 底盘逆解算 - 计算实际自旋角速度
 * @return 底盘实际自旋角速度 (rad/s)
 */
float Chassis_Get_Actual_Omega(void)
{
    // 电机实例数组
    DJI_motor_instance_t *chassis_motor_drive[4] = {chassis_motor_drive_1,
                                                    chassis_motor_drive_2,
                                                    chassis_motor_drive_3,
                                                    chassis_motor_drive_4};

    // 舵向电机零点偏移
    float chassis_motor_direct_zeropoint[4] = {CHASSIS_MOTOR_DIRECT_ZEROPOINT_1,
                                               CHASSIS_MOTOR_DIRECT_ZEROPOINT_2,
                                               CHASSIS_MOTOR_DIRECT_ZEROPOINT_3,
                                               CHASSIS_MOTOR_DIRECT_ZEROPOINT_4};

    DJI_motor_instance_t *chassis_motor_direct[4] = {chassis_motor_direct_1,
                                                     chassis_motor_direct_2,
                                                     chassis_motor_direct_3,
                                                     chassis_motor_direct_4};

    // 轮子位置角度：右前45°，右后-45°，左后-135°，左前135°
    float wheel_position_angle[4] = {PI / 4, -PI / 4, -3 * PI / 4, 3 * PI / 4};

    float omega_sum = 0.0f;
    int valid_count = 0;

    // 遍历四个轮子，计算每个轮子贡献的自旋角速度
    for (int i = 0; i < 4; i++)
    {
        // 获取电机反馈角速度 (rad/s)
        float motor_angular_velocity = chassis_motor_drive[i]->measure.speed;

        // 转换为轮子线速度 (m/s)
        // v = 电机角速度 / 减速比 × 轮子半径
        float wheel_speed = motor_angular_velocity / WHEEL_REDUCTION_RATIO * WHEEL_RADIUS;

        // 获取当前舵向角度
        float current_wheel_angle = chassis_motor_direct[i]->measure.rad - chassis_motor_direct_zeropoint[i];

        // 自旋方向系数（与正解算保持一致）
        float spin_direction = (i == 0 || i == 2) ? -1.0f : 1.0f;

        // 计算轮子速度在切向上的分量
        // 切向方向：垂直于轮子位置的径向方向
        float tangent_angle = wheel_position_angle[i] + PI / 2.0f; // 切向角度

        // 轮子速度在切向上的投影
        float vx = wheel_speed * cosf(current_wheel_angle);
        float vy = wheel_speed * sinf(current_wheel_angle);

        // 切向速度分量
        float v_tangent_x = -sinf(wheel_position_angle[i]);
        float v_tangent_y = cosf(wheel_position_angle[i]);

        // 速度在切向上的投影
        float tangential_speed = vx * v_tangent_x + vy * v_tangent_y;

        // 计算该轮子贡献的自旋角速度: omega = v_tangent / R
        float omega_wheel = spin_direction * tangential_speed / CHASSIS_RADIUS;

        omega_sum += omega_wheel;
        valid_count++;
    }

    // 返回四个轮子的平均自旋角速度
    if (valid_count > 0)
    {
        return omega_sum / valid_count;
    }
    else
    {
        return 0.0f;
    }
}

/**
 * @brief 目标速度斜坡控制函数
 * @param ramp 速度斜坡控制结构体指针
 * @param target 目标速度 (m/s)
 * @return 当前输出速度 (m/s)
 */
float Delta_Target_Speed_Control(Speed_Ramp_t *ramp, float target)
{
    float error = target - ramp->last_speed;
    float step = 0;

    // 根据目标速度与当前输出的关系，判断是加速还是减速，加速步长小于减速步长
    float abs_target = fabs(target);
    float abs_last = fabs(ramp->last_speed);
    step = (target * ramp->last_speed < 0) ? ramp->decel_step : ((abs_target > abs_last) ? ramp->accel_step : ramp->decel_step);
    // 计算出步进后的速度
    ramp->last_speed = (error > step) ? ramp->last_speed + step : ((error < -step) ? ramp->last_speed - step : target);

    return ramp->last_speed;
}

/**
 * @brief 目标角度斜坡控制函数
 * @param pitch_step 角度斜坡步长 (rad)
 * @return 当前输出角度 (rad)
 */
float Delta_Target_Angle_Control(float pitch_step)
{
    if (fabs(target_angle_yaw - target_angle_yaw_temp) >= pitch_step)
    {
        return (target_angle_yaw - target_angle_yaw_temp) >= 0
                   ? (target_angle_yaw_temp + pitch_step)
                   : (target_angle_yaw_temp - pitch_step);
    }
    else
    {
        return target_angle_yaw;
    }
}

void Chassis_State_Machine(void)
{
    // 接收到的数据进行处理
    float target_x_speed = uart2_rx_message.target_x_speed;
    float target_y_speed = uart2_rx_message.target_y_speed;
    float target_omega_speed = uart2_rx_message.target_omega_speed;
    yaw_angle_feedback = uart2_rx_message.INS_yaw;
    yaw_speed_feedback = uart2_rx_message.INS_Gyro_Z;
    chassis_mode = uart2_rx_message.chassis_mode;
    uart2_tx_message.yaw_vel = gimbal_motor_yaw->receive_data.velocity;
    // 速度斜坡控制
    target_x_speed = Delta_Target_Speed_Control(&chassis_x_speed_ramp, target_x_speed);
    target_y_speed = Delta_Target_Speed_Control(&chassis_y_speed_ramp, target_y_speed);
    // 更新 last_speed
    chassis_x_speed_ramp.last_speed = target_x_speed;
    chassis_y_speed_ramp.last_speed = target_y_speed;
    
    if (init_count < 1000)
    {
        init_count++;
        chassis_mode = CHASSIS_MODE_STOP;
        gimbal_angle_yaw_motor = gimbal_motor_yaw->receive_data.position;
    }

    if (chassis_mode == CHASSIS_MODE_AUTO && chassis_mode_last != CHASSIS_MODE_AUTO)
    {
        // 只有在切换的这一秒，让目标等于当前，实现平滑启动
        target_angle_yaw_temp = target_angle_yaw;
    }

    switch (chassis_mode)
    {
    case CHASSIS_MODE_AUTO:

        // 设置云台yaw轴目标角度
        // if (fabs(target_omega_speed) != 0.0f)
        // {
        //     target_angle_yaw -= Chassis_Get_Actual_Omega() / 1035.0f;
        // }
        Chassis_Enable();
        Chassis_Resolving(target_x_speed, target_y_speed, target_omega_speed, gimbal_motor_yaw->receive_data.position);
        //target_angle_yaw_temp = Delta_Target_Angle_Control(0.0007f);

        DM_Motor_SetTar(gimbal_motor_yaw, target_angle_yaw);
        DM_Motor_Control();
        chassis_mode_last = CHASSIS_MODE_AUTO;
        break;

    case CHASSIS_MODE_MANUAL:
        
        // // 设置云台yaw轴目标角度
        // if (fabs(target_omega_speed) != 0.0f)
        // {
        //     target_angle_yaw -= Chassis_Get_Actual_Omega() / 1035.0f;
        // }

        Chassis_Enable();
        Chassis_Resolving(target_x_speed, target_y_speed, target_omega_speed, gimbal_motor_yaw->receive_data.position);
        //target_angle_yaw_temp = Delta_Target_Angle_Control(0.00007f);

        DM_Motor_SetTar(gimbal_motor_yaw, target_angle_yaw);

        DM_Motor_Control();
        chassis_mode_last = CHASSIS_MODE_MANUAL;
        break;

    case CHASSIS_MODE_STOP:
        Chassis_Stop();
        target_angle_yaw = uart2_rx_message.INS_yaw;
        chassis_mode_last = CHASSIS_MODE_STOP;
        break;

    default:
        Chassis_Stop();
        target_angle_yaw = uart2_rx_message.INS_yaw;
        chassis_mode_last = CHASSIS_MODE_STOP;
        break;
    }

    gimbal_angle_yaw_motor_last = gimbal_angle_yaw_motor;             // 更新上次数据值
    gimbal_angle_yaw_motor = gimbal_motor_yaw->receive_data.position; // 获取当前位置
    gimbal_angle_yaw_motor2imu += (gimbal_angle_yaw_motor - gimbal_angle_yaw_motor_last);
    if (fabs(uart2_rx_message.target_omega_speed) >= 0.001f)
    {
        gimbal_angle_yaw_motor2imu -= Chassis_Get_Actual_Omega() / 1035.0f;
    }
    while (gimbal_angle_yaw_motor2imu < -PI)
    {
        gimbal_angle_yaw_motor2imu += 2 * PI;
    }
    while (gimbal_angle_yaw_motor2imu > PI)
    {
        gimbal_angle_yaw_motor2imu -= 2 * PI;
    }

    uart2_tx_message.gimbal_angle_yaw_motor2imu = gimbal_angle_yaw_motor2imu; // 获得用电机解算出的当前yaw角

    DJI_Motor_Control();
}
