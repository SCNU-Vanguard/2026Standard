/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : LPF.c
 * @brief          : lowpass filter
 * @author         : GrassFan Wang
 * @date           : 2025/12/28
 * @version        : v1.0
 ******************************************************************************
 * @attention      : To be perfected
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "lpf.h"

/**
 * @brief 初始化一阶低通滤波器.
 * @param LPF: 一阶低通滤波器结构体.
 * @param Alpha: 滤波器系数.
 * @param Frame_Period: 采样周期.
 * @retval 无.
 */
void LowPass_Filter1p_Init(lowpass_filter1p_info_t *LPF, float Alpha)
{
  LPF->Alpha = Alpha;
  LPF->Input = 0;
  LPF->Output = 0;
  LPF->Initialized = 0;
}

/**
 * @brief 更新一阶低通滤波器数据.
 * @param Input: 当前输入.
 * @retval 滤波后的值.
 */
float LowPass_Filter1p_Update(lowpass_filter1p_info_t *LPF, float Input)
{
    if (Input < 65535 && Input > -65535)
    {

        LPF->Input = Input;

        if (LPF->Initialized == 0)
        {
            LPF->Output = LPF->Input; // 第一次进入更新函数 输出等于这次输入
            LPF->Initialized = 1;
        }

        /*滤波器系数 = Alhpa 0< Alhpa <1 当滤波系数越小，滤波曲线越平稳，但是滞后性更大。
                                         当滤波系数越大，滤波曲线越接近实际值，滞后性小，但是滤波曲线更抖 */

        // 滤波值 = Alhpa * 上一次输出 + （1 - Alhpa)*这次输入
        LPF->Output = LPF->Alpha * LPF->Output + (1.f - LPF->Alpha) * LPF->Input;

        return LPF->Output;
    }
	 return LPF->Output;
}

/**
 * @brief 初始化二阶低通滤波器.
 * @param Alpha[3]: 滤波器系数[3].
 * @retval 无.
 */
void LowPass_Filter2p_Init(lowpass_filter2p_info_t *LPF, float Alpha[3])
{
    memcpy(LPF->Alpha, Alpha, sizeof(LPF->Alpha));
    LPF->Input = 0;
    memset(LPF->Output, 0, sizeof(LPF->Output));
}

/**
 * @brief 更新二阶低通滤波器数据.
 * @param Input: 当前输入.
 * @retval 滤波后的值.
 */
float LowPass_Filter2p_Update(lowpass_filter2p_info_t *LPF, float Input)
{
    if (Input < 1000 && Input > -1000)
    {
        LPF->Input = Input;

        if (LPF->Initialized == 0)
        {
            LPF->Output[0] = LPF->Input;
            LPF->Output[1] = LPF->Input;
            LPF->Output[2] = LPF->Input;
            LPF->Initialized = 1;
        }

        LPF->Output[0] = LPF->Output[1];
        LPF->Output[1] = LPF->Output[2];
        LPF->Output[2] =
            LPF->Alpha[0] * LPF->Output[1] + LPF->Alpha[1] * LPF->Output[0] + LPF->Alpha[2] * LPF->Input;

        return LPF->Output[2];
    }
	return LPF->Output[2];
}

//------------------------------------------------------------------------------
