/**
******************************************************************************
 * @file    procotol_task.c
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

#include "procotol_task.h"
#include "procotol.h"

#include "message_center.h"

#define PROCOTOL_TASK_PERIOD 10 // ms

osThreadId_t procotol_task_handel;

static publisher_t *procotol_publisher;
static subscriber_t *procotol_subscriber;

static void Procotol_Task( void *argument );

void Procotol_Task_Init( void )
{
    const osThreadAttr_t attr = {
            .name = "Procotol_Task",
            .stack_size = 128 * 8,
            .priority = ( osPriority_t )osPriorityRealtime4,
    };
    procotol_task_handel = osThreadNew( Procotol_Task, NULL, &attr );

    procotol_publisher = Publisher_Register("procotol_transmit_feed", 0);
    procotol_subscriber = Subscriber_Register("procotol_receive_cmd", 0);
}

uint32_t procotol_task_diff;

static void Procotol_Task( void *argument )
{
    uint32_t time = osKernelGetTickCount( );

    for( ; ; )
    {
//        VOFA_Display_IMU();
				VOFA_Display_Pitch();
					
        procotol_task_diff = osKernelGetTickCount( ) - time;
        time = osKernelGetTickCount( );
        osDelayUntil( time + PROCOTOL_TASK_PERIOD );
    }
}

