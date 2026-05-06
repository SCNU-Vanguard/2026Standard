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

#include "usart.h"
#include "rs485.h"

#include "super_cap.h"
#define PROCOTOL_TASK_PERIOD 1 // ms

osThreadId_t procotol_task_handel;

static publisher_t *procotol_publisher;
static subscriber_t *procotol_subscriber;

static void Procotol_Task(void *argument);

void Procotol_Task_Init(void)
{
    const osThreadAttr_t attr = {
        .name = "Procotol_Task",
        .stack_size = 128 * 8,
        .priority = (osPriority_t)osPriorityRealtime4,
    };
    procotol_task_handel = osThreadNew(Procotol_Task, NULL, &attr);

    procotol_publisher = Publisher_Register("procotol_transmit_feed", 0);
    procotol_subscriber = Subscriber_Register("procotol_receive_cmd", 0);
}

uint32_t procotol_task_diff;

static void Procotol_Task(void *argument)
{
    // HAL_UART_Receive_IT(&huart1,rev_buf,receive_len);
    uint32_t time = osKernelGetTickCount();
    uint8_t super_cap_tick = 0;

    Super_Cap_instance = Super_Capacitor_Init(&super_cap_init_config);
    for (;;)
    {
        if (++super_cap_tick >= SUPER_CAP_PROCOTOL_DIVIDER)
        {
            super_cap_tick = 0;
            SuperCap_PowerControl_Update();
        }

        procotol_task_diff = osKernelGetTickCount() - time;
        time = osKernelGetTickCount();
        osDelayUntil(time + PROCOTOL_TASK_PERIOD);
    }
}
