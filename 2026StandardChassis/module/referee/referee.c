/*
 * @file		referee.c/h
 * @brief	裁判系统、图传串口中断
 * @history
 * 	版本			作者			编写日期			内容
 * 	v1.0		姚启杰		2023/4/1		裁判系统、图传链路通信
 *	v1.1		郑煜壅		2023/11/3		更改部分代码风格
 *	V2.0		YQJ			2024/4/1		无需再注释库函数
 */
#include "main.h"
#include "usart.h"
#include "dma.h"
#include "string.h"
#include "referee.h"
#include "remote_control.h"
#include "CRC.h"
#include "defense_center.h"

uint8_t referee_rx_len;    //裁判系统串口idle中断接收数据长度
uint8_t referee_pic_rx_len;
uint8_t referee_rx_buf[REFEREE_RXBUFF_SIZE];           //dma接收区
uint8_t referee_pic_rx_buf[REFEREE_PICBUFF_SIZE];		//图传数据
uint8_t referee_tx_buf[REFEREE_TXBUFF_SIZE];                        //maximum 128 bytes

Referee_InfoTypedef refree_info;             //裁判系统数据
uint8_t UI_Seq = 0;

static uint8_t referee_init_flag = 0;
static USART_t *referee_usart_instance;
static supervisor_t *referee_supervisor_instance; 

void RefereeSolve(uint8_t *data);
void RefereeInit()
{
	USART_Service_Init(&huart7);
}

void RefereeSolve(uint8_t *data) //裁判系统的信息处理
{

	uint16_t offset_frame_tail = (data[Offset_SOF_DataLength + 1]<<8) + data[Offset_SOF_DataLength]
							+LEN_CMD_ID +LEN_FRAME_HEAD;
	uint16_t cmd_id = (data[Offset_cmd_ID + 1] << 8 | data[Offset_cmd_ID] );

	if(data[0] != 0xA5 || !Verify_CRC16_Check_Sum(data, offset_frame_tail +2)
			           || !Verify_CRC8_Check_Sum(data, LEN_FRAME_HEAD))
	{
		return;
	}

	switch(cmd_id)
	{
	//0x000
	case ID_game_status:
		memcpy(&(refree_info.Game_Status), (data + Offset_data), LEN_game_state);
		break;
	case ID_game_result:
		memcpy(&(refree_info.Game_Result), (data + Offset_data), LEN_game_result);
		break; 
	case ID_game_robot_hp:
		memcpy(&(refree_info.Game_Robot_HP), (data + Offset_data), LEN_game_robot_hp); //v1.4中有冲突，按sof中为准
		break;

	//0x100
	case ID_event_data:
			memcpy(&(refree_info.Event_Data), (data + Offset_data), LEN_event_data);
			break;
	case ID_referee_warn:
			memcpy(&(refree_info.Referee_Warning), (data + Offset_data), LEN_referee_warn);
			break;
	case ID_dart_shoot_info:
			memcpy(&(refree_info.dart_info), (data + Offset_data), LEN_dart_info);
			break;

	//0x200
	case ID_game_robot_state:
			memcpy(&(refree_info.Game_Robot_state), (data + Offset_data), LEN_game_robot_state);
			break;
	case ID_power_heat_data:
			memcpy(&(refree_info.Power_Heat_Data), (data + Offset_data), LEN_power_heat_data);
			break;
	case ID_game_robot_pos:
			memcpy(&(refree_info.Game_Robot_Pos), (data + Offset_data), LEN_game_robot_pos);
			break;
	case ID_buff_musk:
			memcpy(&(refree_info.Buff_Musk), (data + Offset_data), LEN_buff_musk);
			break;
	case ID_robot_hurt:
			memcpy(&(refree_info.Game_Status), (data + Offset_data), LEN_robot_hurt);
			break;
	case ID_shoot_data:
			memcpy(&(refree_info.Shoot_Data), (data + Offset_data), LEN_shoot_data);//v1.4中冲突，以sof中为准
			break;
	case ID_bullet_remaining:
			memcpy(&(refree_info.bullet_remaining), (data + Offset_data), LEN_bullet_remaining);//v1.4中冲突，以sof中为准
			break;
	case ID_rfid_status:
			memcpy(&(refree_info.rfid_status), (data + Offset_data), LEN_rfid_status);
			break;
	case ID_dart_client_directive:
			memcpy(&(refree_info.dart_client), (data + Offset_data), LEN_dart_client_directive);
			break;

	//0x300
//#ifndef USE_REMOTE_KEYBORAD
//	case ID_keyboard_information:
//			memcpy(&keyboard.keys, (data + Offset_data), LEN_keyboard_information);
//			break;
//#endif
	}

	if(data[offset_frame_tail+2] == 0xA5)
		RefereeSolve(data +offset_frame_tail +2);
	return;

}

static void Referee_Rx_Callback(void)   //接收到数据后的中断回调处理
{
	Supervisor_Reload(referee_supervisor_instance); //喂狗
	RefereeSolve(referee_rx_buf); //裁判系统数据处理
} 


static void Referee_Lost_Callback(void *argument) //裁判系统掉线未正常工作则重启
{
	memset(&refree_info,0,sizeof(refree_info));
	USART_Service_Init(referee_usart_instance);
	(void)argument;
}


Referee_InfoTypedef *Referee_Init(UART_HandleTypeDef *referee_usart_handle) //裁判系统的初始化函数
{
	usart_init_config_t conf;
	conf.module_callback = Referee_Rx_Callback;
	conf.usart_handle    = referee_usart_handle;
	conf.recv_buff_size  = referee_rx_len;
	referee_usart_instance   = USART_Register(&conf);

	// 进行守护进程的注册,用于定时检查裁判系统是否正常工作
	supervisor_init_config_t supervisor_conf = 
	{
		.reload_count = 1000, // 100ms未收到数据视为离线
		.handler_callback = Referee_Lost_Callback,
		.owner_id = NULL, 
	};
	referee_supervisor_instance = Supervisor_Register(&supervisor_conf);

	referee_init_flag = 1;
	return &refree_info;
}







