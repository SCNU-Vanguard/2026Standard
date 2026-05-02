/**
 * @file rm_referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "rm_referee.h"
#include "referee_protocol.h"
#include "CRC.h"

#include "cmsis_os.h"

#if REFEREE_RAW
#else
#define RE_RX_BUFFER_SIZE 255u // 裁判系统接收缓冲区大小
#define TX_MAX_LENGTH 64	 // 裁判系统发送最大数据长度
static USART_t *referee_usart_instance; // 裁判系统串口实例
static supervisor_t *referee_daemon;		  // 裁判系统守护进程
static referee_info_t referee_info;			  // 裁判系统数据

/**
 * @brief  读取裁判数据,中断中读取保证速度
 * @param  buff: 读取到的裁判系统原始数据
 * @retval 是否对正误判断做处理
 * @attention  在此判断帧头和CRC校验,无误再写入数据，不重复判断帧头
 */
static void JudgeReadData(uint8_t *buff)
{
	uint16_t judge_length; // 统计一帧数据长度
	if (buff == NULL)	   // 空数据包，则不作任何处理
		return;

	// 写入帧头数据(5-byte),用于判断是否开始存储裁判数据
	memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);

	// 判断帧头数据(0)是否为0xA5
	if (buff[SOF] == REFEREE_SOF)
	{
		// 帧头CRC8校验
		if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
		{
			// 统计一帧数据长度(byte),用于CRC16校验
			judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;
			// 帧尾CRC16校验
			if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
			{
				// 2个8位拼成16位int
				referee_info.CmdID = (buff[6] << 8 | buff[5]);
				// 解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				// 第8个字节开始才是数据 data=7
				/* 在 JudgeReadData 函数中，替换 switch 分支如下： */
				switch (referee_info.CmdID)
				{
					case ID_game_state:
						memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
						break;
					case ID_game_result:
						memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
						break;
					case ID_game_robot_HP:
						memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
						break;
					case ID_event_data:
						memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
						break;
					case ID_referee_warning:
						memcpy(&referee_info.RefereeWarning, (buff + DATA_Offset), LEN_referee_warning);
						break;
					case ID_dart_info:
						memcpy(&referee_info.DartInfo, (buff + DATA_Offset), LEN_dart_info);
						break;
					case ID_robot_performance:
						memcpy(&referee_info.RobotPerformance, (buff + DATA_Offset), LEN_robot_performance);
						break;
					case ID_power_heat_data:
						memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
						break;
					case ID_game_robot_pos:
						memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
						break;
					case ID_buff_musk:
						memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
						break;
					case ID_robot_hurt:
						memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
						break;
					case ID_shoot_data:
						memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
						break;
					case ID_projectile_allowance:
						memcpy(&referee_info.ProjectileAllowance, (buff + DATA_Offset), LEN_projectile_allowance);
						break;
					case ID_rfid_status:
						memcpy(&referee_info.RFIDStatus, (buff + DATA_Offset), LEN_rfid_status);
						break;
					case ID_dart_launch_status:
						memcpy(&referee_info.DartLaunchStatus, (buff + DATA_Offset), LEN_dart_launch_status);
						break;
					case ID_ground_robot_position:
						memcpy(&referee_info.GroundRobotPosition, (buff + DATA_Offset), LEN_ground_robot_position);
						break;
					case ID_radar_mark_data:
						memcpy(&referee_info.RadarMarkData, (buff + DATA_Offset), LEN_radar_mark_data);
						break;
					case ID_sentry_info:
						memcpy(&referee_info.SentryInfo, (buff + DATA_Offset), LEN_sentry_info);
						break;
					case ID_radar_info:
						memcpy(&referee_info.RadarInfo, (buff + DATA_Offset), LEN_radar_info);
						break;
					case ID_student_interactive:
						memcpy(&referee_info.ReceiveData, (buff + DATA_Offset), LEN_receive_data);
						break;
					case ID_map_command:
						memcpy(&referee_info.MapCommand, (buff + DATA_Offset), LEN_map_command);
						break;
				}
			}
		}
		// 首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,从而判断一个数据包是否有多帧数据
		if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{ // 如果一个数据包出现了多帧数据,则再次调用解析函数,直到所有数据包解析完毕
			JudgeReadData(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
		}
	}
}

/*裁判系统串口接收回调函数,解析数据 */
static void RefereeRxCallback(void)
{
	Supervisor_Reload(referee_daemon);
	JudgeReadData(referee_usart_instance->recv_buff);
}

// 裁判系统丢失回调函数,重新初始化裁判系统串口
static void RefereeLostCallback(void *arg)
{
	USART_Service_Init(referee_usart_instance);
}

/* 裁判系统通信初始化 */
referee_info_t *Referee_Register(UART_HandleTypeDef *referee_usart_handle)
{
	usart_init_config_t conf;
	conf.module_callback   = RefereeRxCallback;
	conf.usart_handle      = referee_usart_handle;
	conf.recv_buff_size    = RE_RX_BUFFER_SIZE; // mx 255(u8)
	referee_usart_instance = USART_Register(&conf);

	supervisor_init_config_t daemon_conf = {
		.handler_callback = RefereeLostCallback,
		.owner_id = referee_usart_instance,
		.reload_count = 1000, // 0.3s没有收到数据,则认为丢失,重启串口接收
	};
	referee_daemon = Supervisor_Register(&daemon_conf);

	return &referee_info;
}

uint8_t uart_test;
/**
 * @brief 裁判系统数据发送函数
 * @param
 */
void Referee_Send(uint8_t *send, uint16_t tx_len)
{
	static uint8_t tx_buffer[TX_MAX_LENGTH];
	// if(USART_Is_Ready(referee_usart_instance))
	// {
	  uart_test = USART_Is_Ready(referee_usart_instance);
		memcpy(tx_buffer, send, tx_len);
		USART_Send(referee_usart_instance, tx_buffer, tx_len, USART_TRANSFER_DMA);
		//USART_Send(referee_usart_instance, send, tx_len, USART_TRANSFER_DMA);
	//}
	//osDelay(40);
}
#endif
