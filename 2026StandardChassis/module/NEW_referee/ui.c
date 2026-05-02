// 接口头文件是ui.h
// 为了使消息能够发送给裁判系统，必须要自定义ui_interface.c中的ui_self_id变量（该变量也可以使用其他代码从裁判系统读取后由程序修改）
// 由于裁判系统接收限制，可能需要使用队列或延时来控制发送频率，此时需要自己建立一个新的发送函数在ui_interface.h中替换掉SEND_MESSAGE的宏定义HAL_UART_Transmit_DMA
#include "ui.h"
#include "math.h"
#include "main.h"

#include "chassis.h"

#include "sentry_info.h"

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "message_center.h"


#include "bsp_dwt.h"

#include "referee_task.h"
#include "robot_frame_init.h"

#define UI_TASK_PERIOD 10 // ms

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t ui_diff;
uint32_t ui_high_water;
#endif

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

// static sentry_msg_t sentry_msg = {0,0,0,0,0,3,0};

osThreadId_t ui_task_handel;
static publisher_t *ui_publisher;
static subscriber_t *ui_subscriber;

static void UI_Task(void *argument);



void UI_Task_Create_Init(void) // 创建所有元素的函数，调用一次即可，如果只能显示部分图案，就尝试重新创建
{
	const osThreadAttr_t attr = {
		.name = "UI_Task",
		.stack_size = 128 * 4,
		.priority = (osPriority_t)osPriorityNormal,
	};

	ui_task_handel = osThreadNew(UI_Task, NULL, &attr);
	ui_publisher = Publisher_Register("ui_transmit_feed", sizeof(ui_behaviour_t));
	ui_subscriber = Subscriber_Register("ui_receive_cmd", sizeof(ui_cmd_t));
}


void UI_Task(void *argument) // 此处根据自己代码的结构体自行更改id及参数
{

	uint32_t time = osKernelGetTickCount();



	for (;;)
	{
		Sentry_Unpacked_Msg(&sentry_get_info);
		EventData_Unpacked_Msg(&event_get_info);
		Referee_Data_Update();
		Sentrycmd_To_Referee();
		ui_diff = osKernelGetTickCount() - time;
		time = osKernelGetTickCount();
		osDelayUntil(time + UI_TASK_PERIOD);
	}
}
