/**
******************************************************************************
* @file    chassis_task.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "chassis_task.h"
#include "chassis.h"

#include "message_center.h"
#include "bsp_dwt.h"

#include "DJI_motor.h"
#include "DM_motor.h"
#include "remote_control.h"
#include "shoot.h"
#define CHASSIS_TASK_PERIOD 1 // ms

osThreadId_t robot_cmd_task_handel;

static publisher_t *chassis_publisher;
static subscriber_t *chassis_subscriber;

static void Chassis_Task(void *argument);

void Chassis_Task_Init(void)
{
    const osThreadAttr_t attr = {
        .name = "Chassis_Task",
        .stack_size = 128 * 8,
        .priority = (osPriority_t)osPriorityRealtime4,
    };
    robot_cmd_task_handel = osThreadNew(Chassis_Task, NULL, &attr);

    chassis_publisher = Publisher_Register("chassis_transmit_feed", sizeof(chassis_behaviour_t));
    chassis_subscriber = Subscriber_Register("chassis_receive_cmd", sizeof(chassis_cmd_t));
}

uint32_t chassis_task_diff;
float TTT = 0;
float AAA = 0;
float speed = 0;
float SP = 0;
float SI = 0;   
float SD = 0;
float SF = 0;
float AP = 0;
float AI = 0;
float AD = 0;
static void Chassis_Task(void *argument)
{
    HAL_UART_Receive_IT(&huart2, &uart2_current_byte, 1);
    uint32_t time = osKernelGetTickCount();

    for (;;)
    {
        // uint32_t Last_time = DWT->CYCCNT;
        Chassis_State_Machine();
         //AAA = gimbal_motor_yaw->receive_data.position;
         AAA = uart2_rx_message.INS_yaw;
         TTT = target_angle_yaw;
        if(TTT > PI) TTT -= 2 * PI;
        if(TTT < -PI) TTT += 2 * PI;
//        gimbal_motor_yaw->motor_controller.speed_PID->kp = SP;
//        gimbal_motor_yaw->motor_controller.speed_PID->ki = SI;
//        gimbal_motor_yaw->motor_controller.speed_PID->kd = SD;
//        gimbal_motor_yaw->motor_controller.angle_PID->kp = AP;
//        gimbal_motor_yaw->motor_controller.angle_PID->ki = AI;
//        gimbal_motor_yaw->motor_controller.angle_PID->kd = AD;
//        gimbal_motor_yaw->motor_controller.angle_PID->kf = SF;

        // chassis_shoot_motor->motor_controller.speed_PID->kp = SP;
        // chassis_shoot_motor->motor_controller.speed_PID->ki = SI;
        // chassis_shoot_motor->motor_controller.speed_PID->kd = SD;
        // chassis_shoot_motor->motor_controller.angle_PID->kp = AP;
        // chassis_shoot_motor->motor_controller.angle_PID->ki = AI;
        // chassis_shoot_motor->motor_controller.angle_PID->kd = AD;

        // speed = uart2_rx_message.INS_Gyro_Z;
        uart2_online_check();

        chassis_task_diff = osKernelGetTickCount() - time;
        time = osKernelGetTickCount();
        osDelayUntil(time + CHASSIS_TASK_PERIOD);

        // chassis_t_ms = DWT_GetTimeline_ms();
        // chassis_delta_t = DWT_GetDeltaT64(&Last_time);
        // chassis_freq = 1 / chassis_delta_t;
    }
}