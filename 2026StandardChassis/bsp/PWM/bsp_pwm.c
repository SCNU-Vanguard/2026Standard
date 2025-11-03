/**
******************************************************************************
 * @file    bsp_pwm.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */
#include <stdlib.h>
#include <string.h>
#include "bsp_pwm.h"

// 配合中断以及初始化
static uint8_t idx;
static PWM_instance_t *pwm_instances[PWM_DEVICE_CNT] = {NULL}; // 所有的pwm instance保存于此,用于callback时判断中断来源
static uint32_t PWM_Select_Tclk(TIM_HandleTypeDef *htim);

/**
 * @brief pwm dma传输完成回调函数
 *
 * @param htim 发生中断的定时器句柄
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	for (uint8_t i = 0 ; i < idx ; i++)
	{ // 来自同一个定时器的中断且通道相同
		if (pwm_instances[i]->htim == htim && htim->Channel == (1 << (pwm_instances[i]->channel / 4)))
		{
			if (pwm_instances[i]->callback) // 如果有回调函数
				pwm_instances[i]->callback(pwm_instances[i]);
			return; // 一次只能有一个通道的中断,所以直接返回
		}
	}
}

PWM_instance_t *PWM_Register(pwm_init_config_t *config)
{
	if (idx >= PWM_DEVICE_CNT) // 超过最大实例数,考虑增加或查看是否有内存泄漏
		while (1);
	PWM_instance_t *pwm = (PWM_instance_t *) malloc(sizeof(PWM_instance_t));
	memset(pwm, 0, sizeof(PWM_instance_t));

	pwm->htim      = config->htim;
	pwm->channel   = config->channel;
	pwm->period    = config->period;
	pwm->dutyratio = config->dutyratio;
	pwm->callback  = config->callback;
	pwm->id        = config->id;
	pwm->tclk      = PWM_Select_Tclk(pwm->htim);
	// 启动PWM
	HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
	PWM_Set_Period(pwm, pwm->period);
	PWM_Set_DutyRatio(pwm, pwm->dutyratio);
	pwm_instances[idx++] = pwm;
	return pwm;
}

/* 只是对HAL的函数进行了形式上的封装 */
void PWM_Start(PWM_instance_t *pwm)
{
	HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
}

/* 只是对HAL的函数进行了形式上的封装 */
void PWM_Stop(PWM_instance_t *pwm)
{
	HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
}

/*
 * @brief 设置pwm周期
 *
 * @param pwm pwm实例
 * @param period 周期 单位 s
 */
void PWM_Set_Period(PWM_instance_t *pwm, float period)
{
	__HAL_TIM_SetAutoreload(pwm->htim, period*((pwm->tclk)/(pwm->htim->Init.Prescaler+1)));
}

/*
    * @brief 设置pwm占空比
    *
    * @param pwm pwm实例
    * @param dutyratio 占空比 0~1
*/
void PWM_Set_DutyRatio(PWM_instance_t *pwm, float dutyratio)
{
	__HAL_TIM_SetCompare(pwm->htim, pwm->channel, dutyratio * (pwm->htim->Instance->ARR));
}

/* 只是对HAL的函数进行了形式上的封装 */
void PWM_Start_DMA(PWM_instance_t *pwm, uint32_t *pData, uint32_t Size)
{
	HAL_TIM_PWM_Start_DMA(pwm->htim, pwm->channel, pData, Size);
}

// 设置pwm对应定时器时钟源频率
//tim2~7,12~14:APB1  tim1,8~11:APB2
static uint32_t PWM_Select_Tclk(TIM_HandleTypeDef *htim)
{
	uintptr_t tclk_temp = ((uintptr_t) (htim->Instance));
	if (
		(tclk_temp <= (APB1PERIPH_BASE + 0x2000UL)) &&
		(tclk_temp >= (APB1PERIPH_BASE + 0x0000UL)))
	{
		return (HAL_RCC_GetPCLK1Freq( ) * (D1CorePrescTable[(RCC->CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos] == 0 ? 1 : 2));
	}
	else if (
		((tclk_temp <= (APB2PERIPH_BASE + 0x0400UL)) &&
		 (tclk_temp >= (APB2PERIPH_BASE + 0x0000UL))) ||
		((tclk_temp <= (APB2PERIPH_BASE + 0x4800UL)) &&
		 (tclk_temp >= (APB2PERIPH_BASE + 0x4000UL))))
	{
		return (HAL_RCC_GetPCLK2Freq( ) * (D1CorePrescTable[(RCC->CFGR & RCC_D2CFGR_D2PPRE1) >> RCC_D2CFGR_D2PPRE1_Pos] == 0 ? 1 : 2));
	}
	return 0;
}
