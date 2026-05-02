/*
 * @Author: GUATAI 2508588132@qq.com
 * @Date: 2026-03-23 14:54:25
 * @LastEditors: GUATAI 2508588132@qq.com
 * @LastEditTime: 2026-03-23 16:12:53
 * @FilePath: \2026_VANGUARD_Frame\module\referee\referee_task.h
 * @Description: 
 * 
 * Copyright (c) 2026 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"

#define REFEREE_RAW 0

#if REFEREE_RAW
#else
extern referee_info_t *referee_outer_info;
extern Referee_Interactive_info_t referee_outer_interactive;
#endif
#if REFEREE_RAW
#else
/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
referee_info_t *UI_Task_Init(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data);

/**
 * @brief 在referee task之前调用,添加在freertos.c中
 * 
 */
void User_UI_Init(void);

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
//void UI_Task(void);

//referee_info_t *UI_Task_Init(UART_HandleTypeDef *referee_usart_handle,
//							 Referee_Interactive_info_t *UI_data);
#endif
#endif // REFEREE_H
