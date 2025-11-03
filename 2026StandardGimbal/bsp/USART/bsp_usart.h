/**
 * @file bsp_usart.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "usart.h"

#define DEVICE_USART_CNT 7     // DM_H7至多分配7个串口
#define USART_RXBUFF_LIMIT 256 // 如果协议需要更大的buff,请修改这里

// 模块回调函数,用于解析协议
typedef void (*usart_module_callback)( );

/* 发送模式枚举 */
typedef enum
{
	USART_TRANSFER_NONE = 0,
	USART_TRANSFER_BLOCKING,
	USART_TRANSFER_IT,
	USART_TRANSFER_DMA,
} usart_transfer_e;

// 串口实例结构体,每个module都要包含一个实例.
// 由于串口是独占的点对点通信,所以不需要考虑多个module同时使用一个串口的情况,因此不用加入id;当然也可以选择加入,这样在bsp层可以访问到module的其他信息
typedef struct
{
	uint8_t current_size;
	uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 预先定义的最大buff大小,如果太小请修改USART_RXBUFF_LIMIT
	uint16_t recv_buff_size;                // 模块接收一包数据的大小
	UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
	usart_module_callback module_callback; // 解析收到的数据的回调函数
	uint16_t beat;
	uint8_t lost_flag;
} USART_t;

/* usart 初始化配置结构体 */
typedef struct
{
	uint16_t recv_buff_size;                // 模块接收一包数据的大小
	UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
	usart_module_callback module_callback; // 解析收到的数据的回调函数
} usart_init_config_t;

USART_t *USART_Register(usart_init_config_t *init_config);

void USART_Service_Init(USART_t *_instance);

void USART_Send(USART_t *_instance, uint8_t *send_buf, uint16_t send_size, usart_transfer_e mode);

uint8_t USART_Error_Lost(USART_t *_instance);

uint8_t USART_Is_Ready(USART_t *_instance);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_USART_H__ */
