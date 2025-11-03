/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       signal_generator.c/h
  * @brief      信号发生器，用于生成测试的各种信号.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2024     Penguin         1. 添加
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#ifndef __SIGNAL_GENERATOR_H__
#define __SIGNAL_GENERATOR_H__

#define M_E 2.7182818284590452354f
#define M_LOG2E 1.4426950408889634074f
#define M_LOG10E 0.43429448190325182765f
#define M_LN2 0.69314718055994530942f
#define M_LN10 2.30258509299404568402f
#define M_PI 3.14159265358979323846f
#define M_PI_2 1.57079632679489661923f
#define M_PI_4 0.78539816339744830962f
#define M_1_PI 0.31830988618379067154f
#define M_2_PI 0.63661977236758134308f
#define M_2_SQRTPI 1.12837916709551257390f
#define M_SQRT2 1.41421356237309504880f
#define M_SQRT1_2 0.70710678118654752440f

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

extern float GenerateSinWave(float amplitude, float offset, float period);

extern float GenerateStepWave(float a0, float a1, float t0);

extern float GenerateRampWave(float a0, float a1, float t0, float t1);

extern float GeneratePulseWave(float a1, float a2, float t1, float t2);

extern float GenerateSawtoothWave(float a0, float a1, float T);

#endif  // __SIGNAL_GENERATOR_H
        /************************ END OF FILE ************************/
