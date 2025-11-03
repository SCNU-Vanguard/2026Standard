/**
******************************************************************************
 * @file    bsp_gpio.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <stdlib.h>
#include <string.h>
#include "bsp_gpio.h"

static uint8_t idx;
static GPIO_instance_t *gpio_instances[GPIO_MX_DEVICE_NUM] = {NULL};

/**
 * @brief EXTI中断回调函数,根据GPIO_Pin找到对应的GPIOInstance,并调用模块回调函数(如果有)
 * @note 如何判断具体是哪一个GPIO的引脚连接到这个EXTI中断线上?
 *       一个EXTI中断线只能连接一个GPIO引脚,因此可以通过GPIO_Pin来判断,PinX对应EXTIX
 *       一个Pin号只会对应一个EXTI,详情见gpio.md
 * @param GPIO_Pin 发生中断的GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// 如有必要,可以根据pinstate和HAL_GPIO_ReadPin来判断是上升沿还是下降沿/rise&fall等
	GPIO_instance_t *gpio;
	for (size_t i = 0 ; i < idx ; i++)
	{
		gpio = gpio_instances[i];
		if (gpio->GPIO_Pin == GPIO_Pin && gpio->gpio_model_callback != NULL)
		{
			gpio->gpio_model_callback(gpio);
			return;
		}
	}
}

GPIO_instance_t *GPIO_Register(gpio_init_config_t *GPIO_config)
{
	GPIO_instance_t *ins = (GPIO_instance_t *) malloc(sizeof(GPIO_instance_t));
	memset(ins, 0, sizeof(GPIO_instance_t));

	ins->GPIOx               = GPIO_config->GPIOx;
	ins->GPIO_Pin            = GPIO_config->GPIO_Pin;
	ins->pin_state           = GPIO_config->pin_state;
	ins->exti_mode           = GPIO_config->exti_mode;
	ins->id                  = GPIO_config->id;
	ins->gpio_model_callback = GPIO_config->gpio_model_callback;
	gpio_instances[idx++]    = ins;
	return ins;
}

// ----------------- GPIO API -----------------
// 都是对HAL的形式上的封装,后续考虑增加GPIO state变量,可以直接读取state

void GPIOToggle(GPIO_instance_t *_instance)
{
	HAL_GPIO_TogglePin(_instance->GPIOx, _instance->GPIO_Pin);
}

void GPIO_Set(GPIO_instance_t *_instance)
{
	HAL_GPIO_WritePin(_instance->GPIOx, _instance->GPIO_Pin, GPIO_PIN_SET);
}

void GPIO_Reset(GPIO_instance_t *_instance)
{
	HAL_GPIO_WritePin(_instance->GPIOx, _instance->GPIO_Pin, GPIO_PIN_RESET);
}

GPIO_PinState GPIO_Read(GPIO_instance_t *_instance)
{
	return HAL_GPIO_ReadPin(_instance->GPIOx, _instance->GPIO_Pin);
}
