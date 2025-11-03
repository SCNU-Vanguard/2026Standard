/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_dwt.h"

DWT_clock_t system_time;
static uint32_t CPU_frequency, CPU_frequency_ms, CPU_frequency_us;
static uint32_t CYCCNT_round_count;
static uint32_t CYCCNT_last;
uint64_t CYCCNT64;

static void DWT_CNT_Update(void)
{
	static volatile uint8_t bit_locker = 0;
	if (!bit_locker)
	{
		bit_locker = 1;

		volatile uint32_t cnt_now = DWT->CYCCNT;

		if (cnt_now < CYCCNT_last)
		{
			CYCCNT_round_count++;
		}

		CYCCNT_last = cnt_now;

		bit_locker = 0;
	}
}

void DWT_Init(uint32_t CPU_Freq_mHz)
{
	/* 使能DWT外设 */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	/* DWT CYCCNT寄存器计数清0 */
	DWT->CYCCNT = (uint32_t) 0u;

	/* 使能Cortex-M DWT CYCCNT寄存器 */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	CPU_frequency      = CPU_Freq_mHz * 1000000;
	CPU_frequency_ms   = CPU_frequency / 1000;
	CPU_frequency_us   = CPU_frequency / 1000000;
	CYCCNT_round_count = 0;

	DWT_CNT_Update( );
}

// 运行总时长
static void DWT_System_Time_Update(void)
{
	volatile uint32_t cnt_now = DWT->CYCCNT;
	static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

	DWT_CNT_Update( );

	CYCCNT64       = (uint64_t) CYCCNT_round_count * (uint64_t) UINT32_MAX + (uint64_t) cnt_now;
	CNT_TEMP1      = CYCCNT64 / CPU_frequency;
	CNT_TEMP2      = CYCCNT64 - CNT_TEMP1 * CPU_frequency;
	system_time.s  = CNT_TEMP1;
	system_time.ms = CNT_TEMP2 / CPU_frequency_ms;
	CNT_TEMP3      = CNT_TEMP2 - system_time.ms * CPU_frequency_ms;
	system_time.us = CNT_TEMP3 / CPU_frequency_us;
}

// 获取秒（float）
float DWT_GetDeltaT(uint32_t *cnt_last)
{
	volatile uint32_t cnt_now = DWT->CYCCNT;
	float dt                  = ((uint32_t) (cnt_now - *cnt_last)) / ((float) (CPU_frequency));
	*cnt_last                 = cnt_now;

	DWT_CNT_Update( );

	return dt;
}

// 获取秒(double)
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
	volatile uint32_t cnt_now = DWT->CYCCNT;
	double dt                 = ((uint32_t) (cnt_now - *cnt_last)) / ((double) (CPU_frequency));
	*cnt_last                 = cnt_now;

	DWT_CNT_Update( );

	return dt;
}

// 获取运行总时长(s)
float DWT_GetTimeline_s(void)
{
	DWT_System_Time_Update( );

	float DWT_Timelinef32 = system_time.s + system_time.ms * 0.001f + system_time.us * 0.000001f;

	return DWT_Timelinef32;
}

// 获取运行总时长(ms)
float DWT_GetTimeline_ms(void)
{
	DWT_System_Time_Update( );

	float DWT_Timelinef32 = system_time.s * 1000 + system_time.ms + system_time.us * 0.001f;

	return DWT_Timelinef32;
}

// 获取运行总时长（us）
uint64_t DWT_GetTimeline_us(void)
{
	DWT_System_Time_Update( );

	uint64_t DWT_Timelinef32 = system_time.s * 1000000 + system_time.ms * 1000 + system_time.us;

	return DWT_Timelinef32;
}

// 延时s
void DWT_Delay(float Delay)
{
	uint32_t tickstart = DWT->CYCCNT;
	float wait         = Delay;

	while ((DWT->CYCCNT - tickstart) < wait * (float) CPU_frequency)
	{
		;
	}
}
