/**
* @file remote_control.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "bsp_usart.h"

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

// 获取按键操作
#define KEY_PRESS 0
#define KEY_STATE 1
#define KEY_PRESS_WITH_CTRL 1
#define KEY_PRESS_WITH_SHIFT 2

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)   // 开关向上时的值
#define RC_SW_MID ((uint16_t)3)  // 开关中间时的值
#define RC_SW_DOWN ((uint16_t)2) // 开关向下时的值
// 三个判断开关状态的宏
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
// 对应key[x][0~16],获取对应的键;例如通过key[KEY_PRESS][Key_W]获取W键是否按下,后续改为位域后删除
#define Key_W 0
#define Key_S 1
#define Key_D 2
#define Key_A 3
#define Key_Shift 4
#define Key_Ctrl 5
#define Key_Q 6
#define Key_E 7
#define Key_R 8
#define Key_F 9
#define Key_G 10
#define Key_Z 11
#define Key_X 12
#define Key_C 13
#define Key_V 14
#define Key_B 15

/* ----------------------- Data Struct ------------------------------------- */
// 待测试的位域结构体,可以极大提升解析速度
typedef union
{
	struct // 用于访问键盘状态
	{
		uint16_t w    : 1;
		uint16_t s    : 1;
		uint16_t d    : 1;
		uint16_t a    : 1;
		uint16_t shift: 1;
		uint16_t ctrl : 1;
		uint16_t q    : 1;
		uint16_t e    : 1;
		uint16_t r    : 1;
		uint16_t f    : 1;
		uint16_t g    : 1;
		uint16_t z    : 1;
		uint16_t x    : 1;
		uint16_t c    : 1;
		uint16_t v    : 1;
		uint16_t b    : 1;
	};

	uint16_t keys; // 用于memcpy而不需要进行强制类型转换
} Key_t;

// @todo 当前结构体嵌套过深,需要进行优化
typedef struct
{
	struct
	{
		int16_t rocker_l_; // 左水平
		int16_t rocker_l1; // 左竖直
		int16_t rocker_r_; // 右水平
		int16_t rocker_r1; // 右竖直
		int16_t dial;      // 侧边拨轮

		uint8_t switch_left;  // 左侧开关
		uint8_t switch_right; // 右侧开关
	} rc;

	struct
	{
		int16_t x;
		int16_t y;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	Key_t key[3]; // 改为位域后的键盘索引,空间减少8倍,速度增加16~倍

	uint8_t key_count[3][16];
} RC_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

RC_ctrl_t *Remote_Control_Init(UART_HandleTypeDef *rc_usart_handle);

uint8_t Remote_Control_Is_Online(void);

#endif /* __REMOTE_CONTROL_H__ */
