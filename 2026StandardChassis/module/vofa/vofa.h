/*
 * @file		vofa.c/h
 * @brief	    vofa+发送数据（调参）
 * @history
 * 	版本			作者			编写日期			内容
 * 	v1.0		冯俊玮		2024/9/10		向上位机vofa+发送数据
 *  v1.1		GUATAI	    2025/8/12		邪恶bsp归一化
 */
#ifndef __VOFA_H__
#define __VOFA_H__

#include <stdint.h>
#include "bsp_usart.h"

extern float vofa_data_view[20];

extern void VOFA_Register(UART_HandleTypeDef *vofa_usart_handle);
extern void VOFA_Send_Data(float *buf, uint8_t len);
extern void VOFA_FireWater(const char *format, ...);
extern void VOFA_JustFloat(float *_data, uint8_t _num);

#endif /* __VOFA_H__ */
