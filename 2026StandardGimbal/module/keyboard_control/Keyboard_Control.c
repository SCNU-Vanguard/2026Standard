/*
 * @file		Keyboard_Control.c/h
 * @brief		键盘控制程序，用于通过键盘输入控制机器人
 * @history
 * 版本			作者			编写日期
 * v1.0.0		miracle		2026/3/18
 * v1.0.1       miracle     2026/3/26  初步完成各种功能的按键对应，如需更改按键更改数组对应字符即可
 */

// application
#include "robot_frame_init.h"

// module
#include "Keyboard_Control.h"
#include "remote_control.h"
#include "remote_vt03.h"

#include "stdbool.h"

/*灵敏度对应图传键鼠灵敏度36*/
#define PITCH_SENSITIVITY 0.000015f // 键盘输入转换为俯仰角的灵敏度调整参数
#define YAW_SENSITIVITY 0.000015f   // 键盘输入转换为底盘速度的灵敏度调整参数

static const float chassis_gear_speed[SPEED_GEAR_COUNT] = {1.5f, 2.0f, 2.5f}; // 不同档位（低，中，高）对应的底盘最大速度
Speed_Gear_e current_gear = GEAR_PRECISION;									  // 默认低速档

float keyboard_speed_x = 0.0f;		// 键盘输入的底盘X方向速度
float keyboard_speed_y = 0.0f;		// 键盘输入的底盘Y方向速度
float keyboard_target_pitch = 0.0f; // 键盘输入的目标俯仰角
float keyboard_target_yaw = 0.0f;	// 键盘输入的目标偏航角
uint8_t Control_Mode = 0;			// 0：遥控器控制 1：键盘控制
uint8_t Omega_flag = 0;				// 小陀螺模式标志位
uint8_t FRIC_flag = 0;				// 摩擦轮启动标志位

/*
 *@brief 更新控制模式
 */
// static void inline Update_Control_Mode()
// {
// 	Control_Mode = (rc_data->key_count[KEY_PRESS][Key_B] % 2); // B键作为键盘控制模式切换的触发键
// }

// static void inline Update_Chassis_Gear()
// {
// 	current_gear = (rc_data->key_count[KEY_PRESS][Key_C] % SPEED_GEAR_COUNT); // C键作为移动档位切换的触发键
// }

// static void inline Update_Chassis_Omega()
// {
// 	Omega_flag = (rc_data->key_count[KEY_PRESS][Key_Z] % 2); // F键作为小陀螺切换的触发键
// }

// static void inline Update_Fire_Ready()
// {
// 	FRIC_flag = (rc_data->key_count[KEY_PRESS][Key_R] % 2); // R键作为发射准备切换的触发键
// }

// static float inline Get_Current_Chassis_Speed_X()
// {
// 	return (rc_data->key[KEY_PRESS].w - rc_data->key[KEY_PRESS].s) * chassis_gear_speed[current_gear]; //x方向速度控制
// }

// static float inline Get_Current_Chassis_Speed_Y()
// {
// 	return (rc_data->key[KEY_PRESS].a - rc_data->key[KEY_PRESS].d) * chassis_gear_speed[current_gear]; //y方向速度控制
// }

// static float inline Get_Current_Target_Pitch()
// {
// 	return (float)rc_data->mouse.y * PITCH_SENSITIVITY; // 将鼠标竖直移动转换为pitch，乘以灵敏度调整参数
// }

// static float inline Get_Current_Target_Yaw()
// {
// 	return (float)rc_data->mouse.x * YAW_SENSITIVITY; // 将鼠标水平移动转换为yaw，乘以灵敏度调整参数
// }

//图传路径控制
static void inline Update_Control_Mode()
{
	Control_Mode = (vt03_data->key_count[KEY_PRESS][Key_B] % 2); // B键作为键盘控制模式切换的触发键
}

static void inline Update_Chassis_Gear()
{
	current_gear = (vt03_data->key_count[KEY_PRESS][Key_C] % SPEED_GEAR_COUNT); // C键作为移动档位切换的触发键
}

static void inline Update_Chassis_Omega()
{
	Omega_flag = (vt03_data->key_count[KEY_PRESS][Key_Z] % 2); // Z键作为小陀螺切换的触发键
}

static void inline Update_Fire_Ready()
{
	FRIC_flag = (vt03_data->key_count[KEY_PRESS][Key_R] % 2); // R键作为发射准备切换的触发键
}

static float inline Get_Current_Chassis_Speed_X()
{
	return (vt03_data->key[KEY_PRESS].w - vt03_data->key[KEY_PRESS].s) * chassis_gear_speed[current_gear]; //x方向速度控制
}

static float inline Get_Current_Chassis_Speed_Y()
{
	return (vt03_data->key[KEY_PRESS].d - vt03_data->key[KEY_PRESS].a) * chassis_gear_speed[current_gear]; //y方向速度控制
}

static float inline Get_Current_Target_Pitch()
{
	return (float)vt03_data->mouse.y * PITCH_SENSITIVITY; // 将鼠标竖直移动转换为pitch，乘以灵敏度调整参数
}

static float inline Get_Current_Target_Yaw()
{
	return (float)vt03_data->mouse.x * YAW_SENSITIVITY; // 将鼠标水平移动转换为yaw，乘以灵敏度调整参数
}

/*
 *@brief 键盘控制主函数
 */
void Keyboard_Control(void)
{
    //切换按键更新
	Update_Control_Mode();
	Update_Chassis_Gear();
	Update_Chassis_Omega();
	Update_Fire_Ready();

	//速度更新
	keyboard_speed_x = Get_Current_Chassis_Speed_X();
	keyboard_speed_y = Get_Current_Chassis_Speed_Y();

	//云台目标角度更新
	keyboard_target_pitch = Get_Current_Target_Pitch();
	keyboard_target_yaw = Get_Current_Target_Yaw();
}