/**
* @file remote_vt03.h
 * @author FJJ (2235038864@qq.com)
 * @brief
 * @version 1.0
 * @date 2026-04-10
 * @copyright Copyright (c) 2026
 *
 */

#ifndef __REMOTE_VT03_H__
#define __REMOTE_VT03_H__

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "bsp_usart.h"

#include "CRC.h"

#define VT03_LAST 1
#define VT03_TEMP 0

// 获取按键操作
#define VT03_KEY_PRESS 0
#define VT03_KEY_STATE 1
#define VT03_KEY_PRESS_WITH_CTRL 1
#define VT03_KEY_PRESS_WITH_SHIFT 2

// 检查接收值是否出错
#define VT03_CH_VALUE_MIN ((uint16_t)364)
#define VT03_CH_VALUE_OFFSET ((uint16_t)1024)
#define VT03_CH_VALUE_MAX ((uint16_t)1684)


/* ----------------------- Data Struct ------------------------------------- */
typedef union
{
	struct // 用于访问键盘状态
	{
		uint16_t w    : 1;
		uint16_t s    : 1;
		uint16_t a    : 1;
		uint16_t d    : 1;
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
} Vt_Key_t;

typedef struct
{
	struct
	{
    int16_t rocker_l_; // 左水平
		int16_t rocker_l1; // 左竖直
		int16_t rocker_r_; // 右水平
		int16_t rocker_r1; // 右竖直
		int16_t dial;      // 侧边拨轮

        //VT03新增的按键
		uint8_t gear_shift;//C:0  N:1  S:2
		uint8_t pause;	   //暂停键 0:未按下 1:按下
		uint8_t auto_key_l;//自动按键左
		uint8_t auto_key_r;//自动按键右
		uint8_t trigge_key;//扳机键
	} rc;


	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;//VT03新增按键,鼠标滚轮速度
		uint8_t press_l;
		uint8_t press_r;
		uint8_t press_m;//鼠标中键 0:未按下 1:按下
	} mouse;
	Vt_Key_t key[3]; /*Keil的Watch对复杂类型解析不稳,观测到与实际数据会不符*/
	uint8_t key_count[3][16];
} VT03_ctrl_t;

/* ------------------------- Internal Data ----------------------------------- */

VT03_ctrl_t *Vt03_Control_Init(UART_HandleTypeDef *vt03_usart_handle);

#endif /* __REMOTE_VT03_H__ */
