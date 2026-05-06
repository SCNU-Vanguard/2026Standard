#include "PowerCtrl.h"
#include "DJI_motor.h" // quote chassis_motor information
#include "referee_task.h"
#include "chassis.h"
#include "robot_frame_init.h"
#include "super_cap.h"

float P_cal[8] = {0};   // 电机功率模型计算出的功率
float P_total = 0;      // 测量总功率
float P_total_temp = 0; // 临时总功率

float P_cmd[8] = {0};         // 以最大功率上线分配所得的功率
float chassis_max_power = 40; // referee_outer_info->RobotPerformance.chassis_power_limit;
float I_temp[8] = {0};

int motor_index = 8; // 参与功率控制的电机数量 (四个底盘轮向电机,四个底盘舵向电机)

power_control_mode_e Power_Control_Mode = SUPERCAP_CONTROL;

#define MIN_DRIVE_POWER 5.0f // 给驱动电机保留的最低底线功率，防止完全抱死

void chassis_power_control(void)
{
    float speed_rpm = 0.0f;
    float a = 0.0f, b = 0.0f, c = 0.0f, delta = 0.0f;
    float E_k = 0.0f;

    // 功率计算相关变量
    float P_cal[8] = {0};
    float P_cmd[4] = {0}; // 只需要给4个驱动电机重新分配
    float I_temp[4] = {0};

    float P_direct_total = 0.0f; // 4个舵向电机总功率
    float P_drive_total = 0.0f;  // 4个轮向电机总功率
    float P_total = 0.0f;        // 整车预测总功率
    // 前4个是M3508驱动电机，后4个是GM6020舵向电机
    DJI_motor_instance_t *chassis_motor_drive[8] = {chassis_motor_drive_1,
                                                    chassis_motor_drive_2,
                                                    chassis_motor_drive_3,
                                                    chassis_motor_drive_4,
                                                    chassis_motor_direct_1,
                                                    chassis_motor_direct_2,
                                                    chassis_motor_direct_3,
                                                    chassis_motor_direct_4};

    // 2. 计算4个舵向电机的功率
    for (uint8_t i = 4; i < 8; i++)
    {
        speed_rpm = chassis_motor_drive[i]->measure.speed * 60.0f; // RAD/S -> RPM

        P_cal[i] = K0_6020 * chassis_motor_drive[i]->target.current * speed_rpm +
                   K1_6020 * speed_rpm * speed_rpm +
                   K2_6020 * chassis_motor_drive[i]->target.current * chassis_motor_drive[i]->target.current +
                   C_6020;

        if (P_cal[i] < 0)
            continue; // 忽略发电负功率
        P_direct_total += P_cal[i];
    }

    // 3. 计算 4个轮向电机(M3508)的预测功率
    for (uint8_t i = 0; i < 4; i++)
    {
        speed_rpm = chassis_motor_drive[i]->measure.speed;

        P_cal[i] = K0_3508 * chassis_motor_drive[i]->target.current * speed_rpm +
                   K1_3508 * speed_rpm * speed_rpm +
                   K2_3508 * chassis_motor_drive[i]->target.current * chassis_motor_drive[i]->target.current +
                   C_3508;

        if (P_cal[i] < 0)
            continue;
        P_drive_total += P_cal[i];
    }

    P_total = P_direct_total + P_drive_total;

    // 4. 功率超限判断与重分配 (仅针对 M3508)
    if (P_total > chassis_max_power)
    {
        float available_drive_power = chassis_max_power - P_direct_total;

        if (available_drive_power < MIN_DRIVE_POWER)
        {
            available_drive_power = MIN_DRIVE_POWER;
        }

        // 计算 驱动电机的功率缩放比例
        float power_scale = available_drive_power / P_drive_total;
        if (power_scale > 1.0f)
            power_scale = 1.0f;

        for (uint8_t i = 0; i < 4; i++)
        {
            speed_rpm = chassis_motor_drive[i]->measure.speed;
            P_cmd[i] = P_cal[i] * power_scale; // 得到驱动电机被分配的合法功率

            if (P_cmd[i] <= 0)
                continue;

            a = K2_3508;
            b = K0_3508 * speed_rpm;
            c = K1_3508 * speed_rpm * speed_rpm + C_3508 - P_cmd[i];
            delta = b * b - 4 * a * c;

            if (delta < 0)
                continue;

            if (chassis_motor_drive[i]->target.current > 0)
            {
                I_temp[i] = (-b + sqrt(delta)) / (2 * a);
                if (I_temp[i] > 16000)
                    chassis_motor_drive[i]->target.current = 16000;
                else
                    chassis_motor_drive[i]->target.current = I_temp[i];
            }
            else
            {
                I_temp[i] = (-b - sqrt(delta)) / (2 * a);
                if (I_temp[i] < -16000)
                    chassis_motor_drive[i]->target.current = -16000;
                else
                    chassis_motor_drive[i]->target.current = I_temp[i];
            }
        }
    }
}

// void Power_Control()
// {
//     if (Power_Control_Mode == SUPERCAP_CONTROL && SuperCap_Data->StatusCode == 3) // 错误，不可恢复
//     {
//         Power_Control_Mode = CODE_CONTROL;
//         SuperCap_SystemRestart(POWER_LIMIT, ENERGY_BUFFER); // 重启超电，尝试一次，不可持续重启
//     }

//     if (SuperCap_Data->StatusCode == 2 || SuperCap_Data->StatusCode == 1) // 错误，可恢复
//     {
//         Power_Control_Mode = CODE_CONTROL;
//         SuperCap_ClearError(POWER_LIMIT, ENERGY_BUFFER); // 清除错误
//     }

//     if (SuperCap_Data->StatusCode == 0)
//     {
//         if (SuperCap_Data->CapEnergy < 64 && Power_Control_Mode == SUPERCAP_CONTROL)
//         {
//             Power_Control_Mode = CODE_CONTROL;
//         }
//         else if (SuperCap_Data->CapEnergy > 128 && Power_Control_Mode == CODE_CONTROL)
//         {
//             Power_Control_Mode = SUPERCAP_CONTROL;
//         }
//     }

//     if (Power_Control_Mode == CODE_CONTROL)
//     {
//         // if(chassis_cmd.mode == SPIN)
//         // {
//         // 	chassis_max_power = 55;
//         // }
//         // else
//         // {
//         chassis_max_power = referee_outer_info->RobotPerformance.chassis_power_limit;
//         chassis_max_power = 20.0f; // 其他模式给一个较低的功率限制，防止过度充电
//         //}
//         // if(SuperCap_Forced_Use_Flag)
//         // {
//         // 	chassis_max_power = 55;
//         // 	if(SuperCap_Data->CapEnergy < 10)
//         // 	{
//         // 		SuperCap_Forced_Use_Flag = 0;
//         // 	}
//         // }

//         chassis_power_control(); // 软件功控
//     }
// }