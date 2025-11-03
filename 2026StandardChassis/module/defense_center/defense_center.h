/**
 * @file defense_center.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef __DEFENSE_CENTER_H__
#define __DEFENSE_CENTER_H__

#include <stdint.h>

#define SUPERVISOR_MAX_CNT 64

typedef void (*offline_callback)(void *id); // 离线回调函数类型,用于处理离线事件

typedef struct
{
	uint16_t reload_count; // 重载计数
	uint16_t temp_count;

	uint16_t online_count; // 在线计数
	uint16_t offline_count; // 离线计数 

	uint8_t online_flag;
	uint8_t offline_flag;

	offline_callback handler_callback; // 离线回调函数
	void *owner_id; // 拥有者ID,可以是模块指针或其他标识符
} supervisor_t;

typedef struct
{
	uint16_t reload_count;
	uint16_t init_count;

	offline_callback handler_callback; // 离线回调函数
	void *owner_id;
} supervisor_init_config_t;

supervisor_t *Supervisor_Register(supervisor_init_config_t *config);

void Supervisor_Reload(supervisor_t *instance);

#endif /* __DEFENSE_CENTER_H__ */
