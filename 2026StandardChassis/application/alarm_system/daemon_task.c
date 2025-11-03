/**
******************************************************************************
 * @file    daemon_task.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h> 

#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "daemon_task.h"
#include "daemon.h"

#include "message_center.h"

#define DAEMON_TASK_PERIOD 10 // ms

osThreadId_t daemon_task_handel;

static publisher_t *daemon_publisher;
static subscriber_t *daemon_subscriber;

static void Daemon_Task( void *argument );

void Daemon_Task_Init( void )
{
    const osThreadAttr_t attr = {
            .name = "Daemon_Task",
            .stack_size = 128 * 8,
            .priority = ( osPriority_t )osPriorityRealtime7,
    };
    daemon_task_handel = osThreadNew( Daemon_Task, NULL, &attr );

    daemon_publisher = Publisher_Register("daemon_transmit_feed", sizeof(daemon_behaviour_t));
    daemon_subscriber = Subscriber_Register("daemon_receive_cmd", sizeof(daemon_cmd_t));
}

uint32_t daemon_task_diff;

static void Daemon_Task( void *argument )
{
    uint32_t time = osKernelGetTickCount( );

    for( ; ; )
    {

        
        daemon_task_diff = osKernelGetTickCount( ) - time;
        time = osKernelGetTickCount( );
        osDelayUntil( time + DAEMON_TASK_PERIOD );
    }
}

osThreadId_t Buzzer_Handle;

void Buzzer_Task(void *argument);

void Buzzer_Task_Init(void)
{
    const osThreadAttr_t attr = {
            .name = "Buzzer_Task",
            .stack_size = 128 * 4,
            .priority = ( osPriority_t )osPriorityNormal,
    };
    Buzzer_Handle = osThreadNew( Buzzer_Task, NULL, &attr );
}

/* USER CODE BEGIN Header__BuzzerTask */
/**
* @brief Function implementing the Buzzer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__BuzzerTask */
__weak void Buzzer_Task(void *argument)
{
  /* USER CODE BEGIN _BuzzerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _BuzzerTask */
}