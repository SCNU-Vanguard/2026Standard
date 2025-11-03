/**
 ******************************************************************************
 * @file	 user_lib.h
 * @author  Wang Hongxi
 * @author  GUATAI
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef __USER_LIB_H__
#define __USER_LIB_H__

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#ifndef user_malloc
#ifdef CMSIS_OS_H_
#define user_malloc pvPortMalloc
#define DELAY_OS 1
#else
#define user_malloc malloc
#endif
#endif

#define FAST_CALC 0

#ifndef user_abs
#define user_abs( x ) ((x > 0) ? x : -x)
#endif

#define msin(x) (arm_sin_f32(x))
#define mcos(x) (arm_cos_f32(x))

typedef arm_matrix_instance_f32 mat;
// 若运算速度不够,可以使用q31代替f32,但是精度会降低
#if FAST_CALC
#define MatAdd arm_mat_add_q31
#define MatSubtract arm_mat_sub_q31
#define MatMultiply arm_mat_mult_q31
#define MatTranspose arm_mat_trans_q31
#define MatInverse arm_mat_inverse_q31
#else
#define MatAdd arm_mat_add_f32
#define MatSubtract arm_mat_sub_f32
#define MatMultiply arm_mat_mult_f32
#define MatTranspose arm_mat_trans_f32
#define MatInverse arm_mat_inverse_f32
#endif
void MatInit(mat *m, uint8_t row, uint8_t col);

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

typedef enum
{
	SLOPE_FIRST_REAL   = 0,
	SLOPE_FIRST_TARGET = 1,
} ramp_strategy_e;

typedef struct
{
	float min_value;    //限幅最小值
	float max_value;    //限幅最大值
	float frame_period; //时间间隔

	float increase_value;
	float decrease_value;

	ramp_strategy_e ramp_state;
} __attribute__((__packed__)) ramp_init_config_t;

typedef struct
{
	float target;       //输入数据
	float out;          //输出数据
	float min_value;    //限幅最小值
	float max_value;    //限幅最大值
	float frame_period; //时间间隔

	float plan_value;
	float real_value;
	float increase_value;
	float decrease_value;
	uint32_t DWT_CNT;
	float dt;

	ramp_strategy_e ramp_state;
} __attribute__((__packed__)) ramp_function_source_t;

typedef struct
{
	uint16_t Order;
	uint32_t Count;

	float *x;
	float *y;

	float k;
	float b;

	float StandardDeviation;

	float t[4];
} __attribute__((__packed__)) Ordinary_Least_Squares_t;

/**
 * @brief 返回一块干净的内存,不过仍然需要强制转换为你需要的类型
 *
 * @param size 分配大小
 * @return void*
 */
void *zmalloc(size_t size);

//快速开方
float Sqrt(float x);

//斜波函数初始化
ramp_function_source_t *ramp_init(ramp_init_config_t *config);

//斜波函数计算
float ramp_calc(ramp_function_source_t *ramp_source_type, float input);

//绝对限制
float abs_limit(float num, float Limit);

//判断符号位
float sign(float value);

//浮点死区
float float_deadband(float Value, float minValue, float maxValue);

//限幅函数
float float_constrain(float Value, float minValue, float maxValue);

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);

//循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue);

//角度 °限幅 180 ~ -180
float theta_format(float Ang);

int float_rounding(float raw);

float *Norm3d(float *v);

float NormOf3d(float *v);

void Cross3d(float *v1, float *v2, float *res);

float Dot3d(float *v1, float *v2);

float AverageFilter(float new_data, float *buf, uint8_t len);

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);

void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);

float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);

float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);

float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS);

float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS);

//判断是否出现NAN
bool Judge_IF_NAN(float x );

#endif /* __USER_LIB_H__ */
