/**
******************************************************************************
 * @file    INS_task.c
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

#include "INS_task.h"
#include "INS.h"

#include "bsp_dwt.h"

#include "message_center.h"

#define INS_TASK_PERIOD 1 // ms

osThreadId_t INS_task_handel;

static publisher_t *INS_publisher;
static subscriber_t *INS_subscriber;

static void INS_Task(void *argument);

void INS_Task_Init(void)
{
	const osThreadAttr_t attr = {
		.name = "INS_Task",
		.stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityRealtime4,
	};
	INS_task_handel = osThreadNew(INS_Task, NULL, &attr);

	INS_publisher  = Publisher_Register("INS_transmit_feed", sizeof(INS_behaviour_t));
	INS_subscriber = Subscriber_Register("INS_receive_cmd", sizeof(INS_cmd_t));
}

uint32_t INS_task_diff;

static void INS_Task(void *argument)
{
	INS_Init( );

	static uint32_t INS_dwt_count = 0;
	float ins_dt                  = 0.0f;

	uint32_t time = osKernelGetTickCount( );

	for (; ;)
	{
		ins_dt = DWT_GetDeltaT(&INS_dwt_count);

		INS_Calculate(ins_dt);

		INS_task_diff = osKernelGetTickCount( ) - time;
		time          = osKernelGetTickCount( );
		osDelayUntil(time + INS_TASK_PERIOD);
	}
}
