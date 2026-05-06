/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "rm_referee.h"
#include "referee_UI.h"

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "rs485.h"

#include "super_cap.h"
#include "remote_control.h"

extern RC_ctrl_t *rc_data;

#if REFEREE_RAW
#else
referee_info_t *referee_outer_info;
Referee_Interactive_info_t referee_outer_interactive;
#endif

#if REFEREE_RAW
#else
static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;			 // 接收到的裁判系统数据
uint8_t UI_Seq;										 // 包序号，供整个referee文件使用
// @todo 不应该使用全局变量

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID(void)
{
	// id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
	referee_recv_info->referee_id.Robot_Color =
		referee_recv_info->RobotPerformance.robot_id > 7 ? Robot_Blue : Robot_Red;
	referee_recv_info->referee_id.Robot_ID = referee_recv_info->RobotPerformance.robot_id;
	referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
	referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

// static void MyUIRefresh(referee_info_t *referee_recv_info,
// 						Referee_Interactive_info_t *_Interactive_data);
// static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
// static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

referee_info_t *UI_Task_Init(UART_HandleTypeDef *referee_usart_handle,
							 Referee_Interactive_info_t *UI_data)
{
	referee_recv_info = Referee_Register(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
	Interactive_data = UI_data;									// 获取UI绘制需要的机器人状态数据
	referee_recv_info->init_flag = 1;
	return referee_recv_info;
}

// void UI_Task(void)
// {
// #if referee_outer_info
// 	RobotModeTest(Interactive_data); // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
// #else
// #endif
// 	MyUIRefresh(referee_recv_info, Interactive_data);
// }

static String_Data_t UI_sign_logo[2];
static Graph_Data_t UI_shoot_line[7]; // 射击准线
static Graph_Data_t UI_energy_line[2];
static Graph_Data_t UI_measure_digital[2];
static String_Data_t UI_state_sta_string[7]; // 机器人状态,静态只需画一次
static Graph_Data_t UI_state_dyn_graph[7];	 // 机器人状态,动态先add才能change
static Graph_Data_t UI_robot_state[7];		 // 机器人状态,动态先add才能change
static uint32_t shoot_line_location[5] = {
	540,
	960,
	490,
	515,
	565,
};

// static void UI_Shoot_Line(void)
// {
// 	// 绘制发射基准线
// 	UI_Line_Draw(&UI_shoot_line[0],
// 				 "sa0",
// 				 UI_Graph_ADD,
// 				 9,
// 				 UI_Color_White,
// 				 3,
// 				 710,
// 				 shoot_line_location[0],
// 				 1210,
// 				 shoot_line_location[0]);
// 	UI_Line_Draw(&UI_shoot_line[1],
// 				 "sa1",
// 				 UI_Graph_ADD,
// 				 9,
// 				 UI_Color_White,
// 				 3,
// 				 shoot_line_location[1],
// 				 340,
// 				 shoot_line_location[1],
// 				 740);
// 	UI_Line_Draw(&UI_shoot_line[2],
// 				 "sa2",
// 				 UI_Graph_ADD,
// 				 9,
// 				 UI_Color_White,
// 				 2,
// 				 985,
// 				 0,
// 				 985,
// 				 shoot_line_location[2]);
// 	UI_Line_Draw(&UI_shoot_line[3],
// 				 "sa3",
// 				 UI_Graph_ADD,
// 				 9,
// 				 UI_Color_White,
// 				 2,
// 				 935,
// 				 0,
// 				 935,
// 				 shoot_line_location[2]);
// 	UI_Line_Draw(&UI_shoot_line[4],
// 				 "sa4",
// 				 UI_Graph_ADD,
// 				 9,
// 				 UI_Color_White,
// 				 2,
// 				 810,
// 				 shoot_line_location[2],
// 				 1110,
// 				 shoot_line_location[2]);
// 	//
// 	UI_Line_Draw(&UI_shoot_line[5],
// 				 "sa5",
// 				 UI_Graph_ADD,
// 				 9,
// 				 UI_Color_Main,
// 				 10,
// 				 250,
// 				 0,
// 				 740,
// 				 625);
// 	UI_Line_Draw(&UI_shoot_line[6],
// 				 "sa6",
// 				 UI_Graph_ADD,
// 				 9,
// 				 UI_Color_Main,
// 				 10,
// 				 1500,
// 				 0,
// 				 1100,
// 				 625);
// 	//

// 	UI_Graph_Refresh(&referee_recv_info->referee_id,
// 					 7,
// 					 UI_shoot_line[0],
// 					 UI_shoot_line[1],
// 					 UI_shoot_line[2],
// 					 UI_shoot_line[3],
// 					 UI_shoot_line[4],
// 					 UI_shoot_line[5],
// 					 UI_shoot_line[6]);
// }

// static void UI_Energy_Line(void)
// {
// 	UI_Arc_Draw(&UI_energy_line[0],
// 				"sb0",
// 				UI_Graph_ADD,
// 				9,
// 				UI_Color_Green,
// 				220,
// 				320,
// 				15,
// 				1000,
// 				540,
// 				425,
// 				425);
// 	UI_Graph_Refresh(&referee_recv_info->referee_id,
// 					 1,
// 					 UI_energy_line[0]);
// }

// static void UI_Measure_Judge_Data(void)
// {
// 	UI_Char_Draw(&UI_sign_logo[0],
// 				 "sc0",
// 				 UI_Graph_ADD,
// 				 8,
// 				 UI_Color_Cyan,
// 				 15,
// 				 2,
// 				 1400,
// 				 750,
// 				 "chs_power:");
// 	UI_Char_Refresh(&referee_recv_info->referee_id, UI_sign_logo[0]);
// 	UI_Float_Draw(&UI_measure_digital[0],
// 				  "sd0",
// 				  UI_Graph_ADD,
// 				  6,
// 				  UI_Color_Main,
// 				  30,
// 				  2,
// 				  3,
// 				  1600,
// 				  760,
// 				  referee_outer_info->ShootData.bullet_speed * 1000);
// 	UI_Graph_Refresh(&referee_recv_info->referee_id,
// 					 1,
// 					 UI_measure_digital[0]);
// }

// static void UI_Robot_State(void)
// {
// 	UI_Arc_Draw(&UI_robot_state[0],
// 				"se0",
// 				UI_Graph_ADD,
// 				9,
// 				UI_Color_Yellow,
// 				330,
// 				10,
// 				10,
// 				960,
// 				540,
// 				300,
// 				300);
// 	UI_Rectangle_Draw(&UI_robot_state[1],
// 					  "se1",
// 					  UI_Graph_ADD,
// 					  9,
// 					  UI_Color_Pink,
// 					  5,
// 					  820,
// 					  400,
// 					  1170,
// 					  690);
// 	UI_Circle_Draw(&UI_robot_state[2],
// 				   "se2",
// 				   UI_Graph_ADD,
// 				   9,
// 				   UI_Color_White,
// 				   5,
// 				   960,
// 				   490,
// 				   10);
// 	UI_Circle_Draw(&UI_robot_state[3],
// 				   "se2",
// 				   UI_Graph_ADD,
// 				   9,
// 				   UI_Color_Main,
// 				   5,
// 				   960,
// 				   490,
// 				   5);
// 	UI_Circle_Draw(&UI_robot_state[4],
// 				   "se2",
// 				   UI_Graph_ADD,
// 				   9,
// 				   UI_Color_Black,
// 				   5,
// 				   960,
// 				   490,
// 				   1);
// 	UI_Graph_Refresh(&referee_recv_info->referee_id,
// 					 5,
// 					 UI_robot_state[0],
// 					 UI_robot_state[1],
// 					 UI_robot_state[2],
// 					 UI_robot_state[3],
// 					 UI_robot_state[4]);
// }

// static void UI_State_Mode_Data(void)
// {
// 	// 绘制车辆状态标志指示
// 	UI_Char_Draw(&UI_state_sta_string[0],
// 				 "ss0",
// 				 UI_Graph_ADD,
// 				 8,
// 				 UI_Color_Cyan,
// 				 15,
// 				 2,
// 				 150,
// 				 750,
// 				 "chassis:");
// 	UI_Char_Refresh(&referee_recv_info->referee_id, UI_state_sta_string[0]);
// 	UI_Char_Draw(&UI_state_sta_string[1],
// 				 "ss1",
// 				 UI_Graph_ADD,
// 				 8,
// 				 UI_Color_Cyan,
// 				 15,
// 				 2,
// 				 150,
// 				 700,
// 				 "gimbal:");
// 	UI_Char_Refresh(&referee_recv_info->referee_id, UI_state_sta_string[1]);
// 	UI_Char_Draw(&UI_state_sta_string[2],
// 				 "ss2",
// 				 UI_Graph_ADD,
// 				 8,
// 				 UI_Color_Cyan,
// 				 15,
// 				 2,
// 				 150,
// 				 650,
// 				 "shoot:");
// 	UI_Char_Refresh(&referee_recv_info->referee_id, UI_state_sta_string[2]);
// 	UI_Char_Draw(&UI_state_sta_string[3],
// 				 "ss3",
// 				 UI_Graph_ADD,
// 				 8,
// 				 UI_Color_Cyan,
// 				 15,
// 				 2,
// 				 150,
// 				 600,
// 				 "frict:");
// 	UI_Char_Refresh(&referee_recv_info->referee_id, UI_state_sta_string[3]);
// 	UI_Char_Draw(&UI_state_sta_string[4],
// 				 "ss4",
// 				 UI_Graph_ADD,
// 				 8,
// 				 UI_Color_Cyan,
// 				 15,
// 				 2,
// 				 150,
// 				 550,
// 				 "ammo:");
// 	UI_Char_Refresh(&referee_recv_info->referee_id, UI_state_sta_string[4]);
// 	UI_Char_Draw(&UI_state_sta_string[5],
// 				 "ss5",
// 				 UI_Graph_ADD,
// 				 8,
// 				 UI_Color_Cyan,
// 				 15,
// 				 2,
// 				 150,
// 				 850,
// 				 "none:");
// 	UI_Char_Refresh(&referee_recv_info->referee_id, UI_state_sta_string[6]);
// 	UI_Char_Draw(&UI_state_sta_string[6],
// 				 "ss6",
// 				 UI_Graph_ADD,
// 				 8,
// 				 UI_Color_Cyan,
// 				 15,
// 				 2,
// 				 150,
// 				 800,
// 				 "control:");
// 	UI_Char_Refresh(&referee_recv_info->referee_id, UI_state_sta_string[6]);

// 	UI_Arc_Draw(&UI_state_dyn_graph[0],
// 				"sr0",
// 				UI_Graph_ADD,
// 				8,
// 				UI_Color_Purplish_red,
// 				0,
// 				360,
// 				5,
// 				275,
// 				790,
// 				10,
// 				10);
// 	UI_Arc_Draw(&UI_state_dyn_graph[1],
// 				"sr1",
// 				UI_Graph_ADD,
// 				8,
// 				UI_Color_Purplish_red,
// 				0,
// 				360,
// 				5,
// 				275,
// 				740,
// 				10,
// 				10);
// 	UI_Arc_Draw(&UI_state_dyn_graph[2],
// 				"sr2",
// 				UI_Graph_ADD,
// 				8,
// 				UI_Color_Purplish_red,
// 				0,
// 				360,
// 				5,
// 				275,
// 				690,
// 				10,
// 				10);
// 	UI_Arc_Draw(&UI_state_dyn_graph[3],
// 				"sr3",
// 				UI_Graph_ADD,
// 				8,
// 				UI_Color_Purplish_red,
// 				0,
// 				360,
// 				5,
// 				275,
// 				640,
// 				10,
// 				10);
// 	UI_Arc_Draw(&UI_state_dyn_graph[4],
// 				"sr4",
// 				UI_Graph_ADD,
// 				8,
// 				UI_Color_Purplish_red,
// 				0,
// 				360,
// 				5,
// 				275,
// 				590,
// 				10,
// 				10);
// 	UI_Arc_Draw(&UI_state_dyn_graph[5],
// 				"sr5",
// 				UI_Graph_ADD,
// 				8,
// 				UI_Color_Purplish_red,
// 				0,
// 				360,
// 				5,
// 				275,
// 				540,
// 				10,
// 				10);
// 	// 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
// 	UI_Arc_Draw(&UI_state_dyn_graph[6],
// 				"sr6",
// 				UI_Graph_ADD,
// 				8,
// 				UI_Color_Main,
// 				0,
// 				360,
// 				5,
// 				20,
// 				900,
// 				10,
// 				10);
// 	UI_Graph_Refresh(&referee_recv_info->referee_id,
// 					 7,
// 					 UI_state_dyn_graph[0],
// 					 UI_state_dyn_graph[1],
// 					 UI_state_dyn_graph[2],
// 					 UI_state_dyn_graph[3],
// 					 UI_state_dyn_graph[4],
// 					 UI_state_dyn_graph[5],
// 					 UI_state_dyn_graph[6]);
// }

// void User_UI_Init()
// {
// 	if (!referee_recv_info->init_flag)
// 	{
// 		osDelay(10000); // 如果没有初始化裁判系统则直接删除ui任务
// 	}
// 	else
// 	{
// 		while (referee_recv_info->RobotPerformance.robot_id == 0)
// 		{
// 			osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查
// 		}

// 		DeterminRobotID();											   // 确定ui要发送到的目标客户端
// 		UI_Delete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

// 		UI_Shoot_Line();

// 		UI_State_Mode_Data();

// 		UI_Energy_Line();

// 		UI_Measure_Judge_Data();

// 		UI_Robot_State();
// 	}
// }

// // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
// static uint8_t count = 0;
// static uint16_t count1 = 0;
// static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
// {
// 	count++;
// 	if (count >= 50)
// 	{
// 		count = 0;
// 		count1++;
// 	}
// 	switch (count1 % 2)
// 	{
// 	case 0:
// 	{
// 		//   _Interactive_data->chassis_mode = CHASSIS_DISABLE;
// 		//   _Interactive_data->gimbal_mode = GIMBAL_DISABLE;
// 		//   _Interactive_data->shoot_mode = SHOOT_DISABLE;
// 		//   _Interactive_data->friction_mode = 0;
// 		//   _Interactive_data->ammo_mode = 0;
// 		_Interactive_data->Chassis_Power_Limit += 1.5;
// 		_Interactive_data->Super_Power += 1.5;
// 		if (_Interactive_data->Chassis_Power_Limit >= 70)
// 		{
// 			_Interactive_data->Super_Power = 0;
// 			_Interactive_data->Chassis_Power_Limit = 0;
// 		}
// 		break;
// 	}
// 	case 1:
// 	{
// 		//   _Interactive_data->chassis_mode = CHASSIS_FOLLOW;
// 		//   _Interactive_data->gimbal_mode = GIMBAL_ENABLE;
// 		//   _Interactive_data->shoot_mode = SHOOT_ENABLE;
// 		//   _Interactive_data->friction_mode = 1;
// 		//   _Interactive_data->ammo_mode = 0;
// 		_Interactive_data->Chassis_Power_Limit -= 0.1;
// 		_Interactive_data->Super_Power -= 0.1;
// 		if (_Interactive_data->Chassis_Power_Limit <= 0)
// 		{
// 			_Interactive_data->Chassis_Power_Limit = 25;
// 			_Interactive_data->Super_Power = 25;
// 		}
// 		break;
// 	}
// 	case 2:
// 	{
// 		//   _Interactive_data->chassis_mode = CHASSIS_SPIN;
// 		//   _Interactive_data->gimbal_mode = GIMBAL_AUTO_AIMING;
// 		//   _Interactive_data->shoot_mode = SHOOT_AUTO_AIMING;
// 		//   _Interactive_data->friction_mode = 0;
// 		//   _Interactive_data->ammo_mode = 1;
// 		_Interactive_data->Chassis_Power_Limit += 1.5;
// 		_Interactive_data->Super_Power += 1.5;
// 		if (_Interactive_data->Chassis_Power_Limit >= 70)
// 		{
// 			_Interactive_data->Super_Power = 0;
// 			_Interactive_data->Chassis_Power_Limit = 0;
// 		}
// 		break;
// 	}
// 	case 3:
// 	{
// 		//   _Interactive_data->chassis_mode = CHASSIS_UPSTEP;
// 		//   _Interactive_data->gimbal_mode = GIMBAL_DISABLE;
// 		//   _Interactive_data->shoot_mode = SHOOT_DISABLE;
// 		//   _Interactive_data->friction_mode = 1;
// 		//   _Interactive_data->ammo_mode = 1;
// 		_Interactive_data->Chassis_Power_Limit -= 0.1;
// 		_Interactive_data->Super_Power -= 0.1;
// 		if (_Interactive_data->Chassis_Power_Limit <= 0)
// 		{
// 			_Interactive_data->Chassis_Power_Limit = 25;
// 			_Interactive_data->Super_Power = 25;
// 		}
// 		break;
// 	}
// 	default:
// 		break;
// 	}
// }

// static void MyUIRefresh(referee_info_t *referee_recv_info,
// 						Referee_Interactive_info_t *_Interactive_data)
// {
// 	static uint32_t refresh_cnt;
// 	static uint8_t refresh_flag;

// 	static uint8_t error_code = 0;
// 	static uint8_t error_last_code = 0;
// 	static uint8_t error_flag = 0;

// 	refresh_flag = 0;
// 	refresh_cnt++;

// 	//   _Interactive_data->Chassis_Power_Limit = referee_outer_info->RobotPerformance.chassis_power_limit;
// //	if (rc_data->online == 0)
// //	{
// //		_Interactive_data->control_mode = 2; // control_cmd.control;
// //	}
// //	else
// //	{
// //		_Interactive_data->control_mode = chassis_cmd.key_state.key_EN_state; // control_cmd.control;
// //	}
	
	
// 	if (chassis_shoot_motor->target.current < -50)
// 	{
// 		_Interactive_data->ammo_mode = 1; // gimbal_message_chat.ammo;
// 	}
// 	else if(chassis_shoot_motor->target.current == -9000)
// 	{
// 		_Interactive_data->ammo_mode = 2; // gimbal_message_chat.ammo;
// 	}
// 	else
// 	{
// 		_Interactive_data->ammo_mode = 0; // gimbal_message_chat.ammo;
// 	}

// 	_Interactive_data->friction_mode = friction_state; // gimbal_message_chat.friction_state;
// 	_Interactive_data->chassis_mode = chassis_cmd.mode;					// chassis_cmd.chassis;
// 	_Interactive_data->gimbal_mode = gimbal_cmd.ctrl_mode;					// gimbal_cmd.gimbal;
// 	_Interactive_data->shoot_mode = shoot_mode;						// shoot_cmd.shooter;
// 	_Interactive_data->Super_Power = trans_thresholds(SuperCap_Data->CapEnergy, 0, 255, 0, 100);
// 	_Interactive_data->Chassis_Power_Limit = SuperCap_Data->ChassisPower;

// 	//   RobotModeTest(Interactive_data); // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常

// 	uint16_t zero_UI_point = 0;
// 	zero_UI_point = trans_zeropoint(DM_6006_yaw -> receive_data.position, ANGLE_REFERENCE, 0, 2 * PI, 0, 360);

// 	UIChangeCheck(_Interactive_data);

// 	if (_Interactive_data->Referee_Interactive_Flag.control_flag == 1)
// 	{
// 		switch (_Interactive_data->control_mode)
// 		{
// 		case 0: // CONTROL_REMOTE:
// 			UI_Arc_Draw(&UI_state_dyn_graph[0],
// 						"sr0",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Yellow,
// 						0,
// 						360,
// 						5,
// 						275,
// 						790,
// 						10,
// 						10);
// 			// 此处注意字数对齐问题，字数相同才能覆盖掉
// 			break;
// 		case 1: // CONTROL_PC:
// 			UI_Arc_Draw(&UI_state_dyn_graph[0],
// 						"sr0",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Green,
// 						0,
// 						360,
// 						5,
// 						275,
// 						790,
// 						10,
// 						10);
// 			break;
// 		case 2: // CONTROL_NONE:
// 			UI_Arc_Draw(&UI_state_dyn_graph[0],
// 						"sr0",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Purplish_red,
// 						0,
// 						360,
// 						5,
// 						275,
// 						790,
// 						10,
// 						10);
// 			break;
// 		}
// 		UI_Graph_Refresh(&referee_recv_info->referee_id,
// 						 1,
// 						 UI_state_dyn_graph[0]);

// 		_Interactive_data->Referee_Interactive_Flag.control_flag = 0;
// 	}

// 	// chassis
// 	if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
// 	{
// 		switch (_Interactive_data->chassis_mode)
// 		{
// 		case STOP_C:
// 			UI_Arc_Draw(&UI_state_dyn_graph[1],
// 						"sr1",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Purplish_red,
// 						0,
// 						360,
// 						5,
// 						275,
// 						740,
// 						10,
// 						10);
// 			break;
// 		case FOLLOW:
// 			UI_Arc_Draw(&UI_state_dyn_graph[1],
// 						"sr1",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Green,
// 						0,
// 						360,
// 						5,
// 						275,
// 						740,
// 						10,
// 						10);
// 			// 此处注意字数对齐问题，字数相同才能覆盖掉
// 			break;
// 		default:
// 			UI_Arc_Draw(&UI_state_dyn_graph[1],
// 						"sr1",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Orange,
// 						0,
// 						360,
// 						5,
// 						275,
// 						740,
// 						10,
// 						10);
// 			break;
// 		}
// 		UI_Graph_Refresh(&referee_recv_info->referee_id,
// 						 1,
// 						 UI_state_dyn_graph[1]);

// 		_Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
// 	}
// 	// gimbal
// 	if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
// 	{
// 		switch (_Interactive_data->gimbal_mode)
// 		{
// 		case GIMBAL_DISABLE:
// 		{
// 			UI_Arc_Draw(&UI_state_dyn_graph[2],
// 						"sr2",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Purplish_red,
// 						0,
// 						360,
// 						5,
// 						275,
// 						690,
// 						10,
// 						10);
// 			break;
// 		}
// 		case GIMBAL_ENABLE: // GIMBAL_NORMAL:
// 		{
// 			UI_Arc_Draw(&UI_state_dyn_graph[2],
// 						"sr2",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Green,
// 						0,
// 						360,
// 						5,
// 						275,
// 						690,
// 						10,
// 						10);
// 			break;
// 		}
// 		default: // GIMBAL_AUTO:
// 		{
// 			UI_Arc_Draw(&UI_state_dyn_graph[2],
// 						"sr2",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Orange,
// 						0,
// 						360,
// 						5,
// 						275,
// 						690,
// 						10,
// 						10);
// 			break;
// 		}
// 		}
// 		UI_Graph_Refresh(&referee_recv_info->referee_id,
// 						 1,
// 						 UI_state_dyn_graph[2]);
// 		_Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
// 	}
// 	// shoot
// 	if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
// 	{
// 		switch (_Interactive_data->shoot_mode)
// 		{
// 		case SHOOT_MODE_STOP:
// 			UI_Arc_Draw(&UI_state_dyn_graph[3],
// 						"sr3",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Purplish_red,
// 						0,
// 						360,
// 						5,
// 						275,
// 						640,
// 						10,
// 						10);
// 			break;
// 		case SHOOT_MODE_FIRE:
// 			UI_Arc_Draw(&UI_state_dyn_graph[3],
// 						"sr3",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Green,
// 						0,
// 						360,
// 						5,
// 						275,
// 						640,
// 						10,
// 						10);
// 			break;
// 		default:
// 			UI_Arc_Draw(&UI_state_dyn_graph[3],
// 						"sr3",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Orange,
// 						0,
// 						360,
// 						5,
// 						275,
// 						640,
// 						10,
// 						10);
// 			break;
// 		}
// 		UI_Graph_Refresh(&referee_recv_info->referee_id,
// 						 1,
// 						 UI_state_dyn_graph[3]);
// 		_Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
// 	}
// 	// friction
// 	if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
// 	{
// 		switch (_Interactive_data->friction_mode)
// 		{
// 		case 0:
// 			UI_Arc_Draw(&UI_state_dyn_graph[4],
// 						"sr4",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Purplish_red,
// 						0,
// 						360,
// 						5,
// 						275,
// 						590,
// 						10,
// 						10);
// 			break;
// 		case 1:
// 			UI_Arc_Draw(&UI_state_dyn_graph[4],
// 						"sr4",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Green,
// 						0,
// 						360,
// 						5,
// 						275,
// 						590,
// 						10,
// 						10);
// 			break;
// 		}
// 		UI_Graph_Refresh(&referee_recv_info->referee_id,
// 						 1,
// 						 UI_state_dyn_graph[4]);
// 		_Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
// 	}
// 	// ammo
// 	if (_Interactive_data->Referee_Interactive_Flag.ammo_flag == 1)
// 	{
// 		switch (_Interactive_data->ammo_mode)
// 		{
// 		case 0:
// 			UI_Arc_Draw(&UI_state_dyn_graph[5],
// 						"sr5",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Purplish_red,
// 						0,
// 						360,
// 						5,
// 						275,
// 						540,
// 						10,
// 						10);
// 			break;
// 		case 1:
// 			UI_Arc_Draw(&UI_state_dyn_graph[5],
// 						"sr5",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Green,
// 						0,
// 						360,
// 						5,
// 						275,
// 						540,
// 						10,
// 						10);
// 			break;
// 		case 2:
// 			UI_Arc_Draw(&UI_state_dyn_graph[5],
// 						"sr5",
// 						UI_Graph_Change,
// 						8,
// 						UI_Color_Orange,
// 						0,
// 						360,
// 						5,
// 						275,
// 						540,
// 						10,
// 						10);
// 			break;
// 		}
// 		UI_Graph_Refresh(&referee_recv_info->referee_id,
// 						 1,
// 						 UI_state_dyn_graph[5]);
// 		_Interactive_data->Referee_Interactive_Flag.ammo_flag = 0;
// 	}

// 	error_code = 0; // super_cap->receive_data.errorCode;
// 	if (error_code != error_last_code)
// 	{
// 		error_flag = 1;
// 	}
// 	else
// 	{
// 		error_flag = 0;
// 	}

// 	if ((refresh_cnt % 3) == 0)
// 	{
// 		uint16_t left_step, right_step;
// 		left_step = int16_constrain((zero_UI_point - 15), 0, 360);
// 		right_step = int16_constrain((zero_UI_point + 15), 0, 360);
// 		UI_Arc_Draw(&UI_robot_state[0],
// 					"se0",
// 					UI_Graph_Change,
// 					9,
// 					UI_Color_Yellow,
// 					left_step,
// 					right_step,
// 					10,
// 					960,
// 					540,
// 					300,
// 					300);
// 		UI_Graph_Refresh(&referee_recv_info->referee_id,
// 						 1,
// 						 UI_robot_state[0]);
// 	}

// 	if ((refresh_cnt & 4) == 0)
// 	{
// 		if (1)
// 		{
// 			;
// 		}
// 		else
// 		{
// 			;
// 		}
// 	}

// 	if ((refresh_cnt % 5) == 0)
// 	{
// 		// power
// 		if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)
// 		{
// 			UI_Float_Draw(&UI_measure_digital[0],
// 						  "sd0",
// 						  UI_Graph_Change,
// 						  6,
// 						  UI_Color_Main,
// 						  30,
// 						  2,
// 						  3,
// 						  1600,
// 						  760,
// 						  (_Interactive_data->Chassis_Power_Limit * 1000));
// 			UI_Graph_Refresh(&referee_recv_info->referee_id,
// 							 1,
// 							 UI_measure_digital[0]);
// 		}
// 	}

// 	if ((refresh_cnt % 6) == 0)
// 	{
// 		if (_Interactive_data->Referee_Interactive_Flag.Super_flag == 1)
// 		{
// 			if (_Interactive_data->Super_Power > 75)
// 			{
// 				UI_Arc_Draw(&UI_energy_line[0],
// 							"sb0",
// 							UI_Graph_Change,
// 							9,
// 							UI_Color_Green,
// 							220,
// 							220 + _Interactive_data->Super_Power,
// 							15,
// 							1000,
// 							540,
// 							425,
// 							425);
// 			}
// 			else if (_Interactive_data->Super_Power > 30)
// 			{
// 				UI_Arc_Draw(&UI_energy_line[0],
// 							"sb0",
// 							UI_Graph_Change,
// 							9,
// 							UI_Color_Orange,
// 							220,
// 							220 + _Interactive_data->Super_Power,
// 							15,
// 							1000,
// 							540,
// 							425,
// 							425);
// 			}
// 			else
// 			{
// 				UI_Arc_Draw(&UI_energy_line[0],
// 							"sb0",
// 							UI_Graph_Change,
// 							9,
// 							UI_Color_Purplish_red,
// 							220,
// 							220 + _Interactive_data->Super_Power,
// 							15,
// 							1000,
// 							540,
// 							425,
// 							425);
// 			}
// 			UI_Graph_Refresh(&referee_recv_info->referee_id,
// 							 1,
// 							 UI_energy_line[0]);
// 		}
// 	}

// 	error_last_code = error_code;
// }

// /**
//  * @brief  模式切换检测,模式发生切换时，对flag置位
//  * @param  Referee_Interactive_info_t *_Interactive_data
//  * @retval none
//  * @attention
//  */
// static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
// {
// 	if (_Interactive_data->control_mode != _Interactive_data->control_last_mode)
// 	{
// 		_Interactive_data->Referee_Interactive_Flag.control_flag = 1;
// 		_Interactive_data->control_last_mode = _Interactive_data->control_mode;
// 	}

// 	if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
// 	{
// 		_Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
// 		_Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
// 	}

// 	if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
// 	{
// 		_Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
// 		_Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
// 	}

// 	if (_Interactive_data->shoot_mode != _Interactive_data->shoot_last_mode)
// 	{
// 		_Interactive_data->Referee_Interactive_Flag.shoot_flag = 1;
// 		_Interactive_data->shoot_last_mode = _Interactive_data->shoot_mode;
// 	}

// 	if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
// 	{
// 		_Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
// 		_Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
// 	}

// 	if (_Interactive_data->ammo_mode != _Interactive_data->ammo_last_mode)
// 	{
// 		_Interactive_data->Referee_Interactive_Flag.ammo_flag = 1;
// 		_Interactive_data->ammo_last_mode = _Interactive_data->ammo_mode;
// 	}
// 	if (_Interactive_data->Chassis_Power_Limit != _Interactive_data->Chassis_last_Power_Limit)
// 	{
// 		_Interactive_data->Referee_Interactive_Flag.Power_flag = 1;
// 		_Interactive_data->Chassis_last_Power_Limit = _Interactive_data->Chassis_Power_Limit;
// 	}
// 	if (_Interactive_data->Super_Power != _Interactive_data->Super_last_Power)
// 	{
// 		if (user_abs(_Interactive_data->Super_Power - _Interactive_data->Super_last_Power) < 1)
// 		{
// 			_Interactive_data->Referee_Interactive_Flag.Super_flag = 1;
// 		}
// 		_Interactive_data->Super_last_Power = _Interactive_data->Super_Power;
// 	}
// }
 #endif