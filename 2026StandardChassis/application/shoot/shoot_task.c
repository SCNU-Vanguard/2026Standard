/**
******************************************************************************
 * @file    shoot_task.c
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

#include "shoot_task.h" 
#include "shoot.h"

#include "message_center.h"

#include "DJI_motor.h"
#include "remote_control.h"

#define SHOOT_TASK_PERIOD 5 // ms

osThreadId_t shoot_task_handel;

static publisher_t *shoot_publisher;
static subscriber_t *shoot_subscriber;

static void Shoot_Task( void *argument );

void Shoot_Task_Init( void )
{
    const osThreadAttr_t attr = {
            .name = "Shoot_Task",
            .stack_size = 128 * 8,
            .priority = ( osPriority_t )osPriorityRealtime3,
    };
    shoot_task_handel = osThreadNew( Shoot_Task, NULL, &attr );

    shoot_publisher = Publisher_Register("shoot_transmit_feed", sizeof(shoot_behaviour_t));
    shoot_subscriber = Subscriber_Register("shoot_receive_cmd", sizeof(shoot_cmd_t));
}

uint32_t shoot_task_diff;

static void Shoot_Task( void *argument )
{
    uint32_t time = osKernelGetTickCount( );

    for( ; ; )
    {
        Shoot_State_Machine();
        shoot_task_diff = osKernelGetTickCount( ) - time;
        time = osKernelGetTickCount( );
        osDelayUntil( time + SHOOT_TASK_PERIOD );
    }
}