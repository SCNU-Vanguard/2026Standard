// 接口头文件是ui.h
// 为了使消息能够发送给裁判系统，必须要自定义ui_interface.c中的ui_self_id变量（该变量也可以使用其他代码从裁判系统读取后由程序修改）
// 由于裁判系统接收限制，可能需要使用队列或延时来控制发送频率，此时需要自己建立一个新的发送函数在ui_interface.h中替换掉SEND_MESSAGE的宏定义HAL_UART_Transmit_DMA
#include "ui.h"
#include "math.h"
#include "main.h"
#include "gimbal.h"
#include "remote_control.h"
#include "chassis.h"
#include "referee.h"
#include "ui_interface.h"

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "message_center.h"

#include "DM_motor.h"

#include "bsp_dwt.h"
#include "rs485.h"
#include "referee.h"
#include "robot_frame_init.h"

#define UI_TASK_PERIOD 1 // ms

#if INCLUDE_uxTaskGetStackHighWaterMark
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

uint32_t ui_diff = 0;
uint8_t fric_flag;
uint8_t auto_flag;

uint8_t supercap_energy = 0;
float delta_gim;

void UI_Task_Init(void) // 创建所有元素的函数，调用一次即可，如果只能显示部分图案，就尝试重新创建
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

void Referee_Data_Update()
{
	uart2_tx_message.game_time = referee_data->Game_Status.stage_remain_time;  // 获取当前阶段剩余时间
	uart2_tx_message.is_play = referee_data->Game_Status.game_progress;		  // 当前比赛阶段
	uart2_tx_message.own_hp[0] = referee_data->Game_Robot_HP.ally_7_robot_HP; // 获取哨兵机器人HP
	uart2_tx_message.own_hp[1] = referee_data->Game_Robot_HP.ally_1_robot_HP; // 获取英雄机器人HP
	uart2_tx_message.own_hp[2] = referee_data->Game_Robot_HP.ally_3_robot_HP; // 获取步兵机器人HP
	uart2_tx_message.event_data = (referee_data->Event_Data.event_data >> 23) & 0x03;  //场地状态
	uart2_tx_message.enemy_score = 0;
	uart2_tx_message.own_score = 0;
	
}

void UI_Task(void *argument) // 此处根据自己代码的结构体自行更改id及参数
{

	uint32_t time = osKernelGetTickCount();



	for (;;)
	{
		Auto_sentrycmd_set();
		Referee_Data_Update();
		ui_diff = osKernelGetTickCount() - time;
		time = osKernelGetTickCount();
		osDelayUntil(time + UI_TASK_PERIOD);
		// #if INCLUDE_uxTaskGetStackHighWaterMark
		// 		ui_high_water = uxTaskGetStackHighWaterMark(NULL);
		// #endif
	}
}
