/**
******************************************************************************
 * @file    denfense_center.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */
#include <stdlib.h>
#include <string.h>
#include "defense_center.h"

// 用于保存所有的supervisor instance
static supervisor_t *supervisor_instances[SUPERVISOR_MAX_CNT] = {NULL};
static uint8_t idx; // 用于记录当前的supervisor instance数量,配合回调使用

/**
 * @brief 注册一个supervisor实例
 *
 * @param config 初始化配置
 * @return supervisor_t* 返回实例指针
 */
supervisor_t *Supervisor_Register(supervisor_init_config_t *config)
{
	supervisor_t *instance = (supervisor_t *) malloc(sizeof(supervisor_t));
	memset(instance, 0, sizeof(supervisor_t));

	instance->owner_id         = config->owner_id;
	instance->reload_count     = config->reload_count; // 默认值为100
	instance->handler_callback = config->handler_callback;
	instance->temp_count       = config->init_count; // 默认值为100,初始计数

	supervisor_instances[idx++] = instance;
	return instance;
}

/* "喂狗"函数 */
void Supervisor_Reload(supervisor_t *instance)
{
	instance->temp_count = instance->reload_count;
}

/**
 * @brief 会给每个supervisor实例的temp_count按频率进行递减操作.
 *        模块成功接受数据或成功操作则会重载temp_count的值为reload_count.
 *
 */
void Supervisor_Task(void)
{
	supervisor_t *dins; // 提高可读性同时降低访存开销
	for (size_t i = 0 ; i < idx ; ++i)
	{
		dins = supervisor_instances[i];
		if (dins->temp_count > 0) // 如果计数器还有值,说明上一次喂狗后还没有超时,则计数器减一
		{
			dins->offline_count = 0;
			dins->online_count++;
			dins->temp_count--;
			if ((dins->online_flag == 0) && (dins->online_count >= 10))
			{
				dins->online_flag  = 1; // 标记在线状态
				dins->offline_flag = 0; // 清除离线标记
			}
		}
		else // 等于零说明超时了,调用回调函数(如果有的话)
		{
			dins->online_count = 0;
			dins->offline_count++;
			if ((dins->offline_flag == 0) && (dins->offline_count >= 10))
			{
				dins->offline_flag = 1; // 标记离线状态
				dins->online_flag  = 0; // 清除在线标记
			}

			if (dins->offline_flag == 1)
			{
				if (dins->handler_callback)
				{
					dins->handler_callback(dins->owner_id); // module内可以将owner_id强制类型转换成自身类型从而调用特定module的offline callback
				}
			}
			// @todo 为蜂鸣器/led等增加离线报警的功能,非常非常非常关键!
		}
	}
}
