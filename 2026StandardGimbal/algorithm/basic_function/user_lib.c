/**
 ******************************************************************************
 * @file	 user_lib.c
 * @author  Wang Hongxi
 * @author  modified by neozng
 * @author  GUATAI
 * @version 0.2 beta
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "user_lib.h"

#ifdef CMSIS_OS_H_
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

/**
 * @brief          斜波函数初始化
 * @author         GUATAI
 * @param[in]      斜波函数结构体
 * @param[in]      间隔的时间，单位 s
 * @param[in]      最大值
 * @param[in]      最小值
 * @retval         返回空
 */
ramp_function_source_t* ramp_init(ramp_init_config_t *config)
{
  ramp_function_source_t *ramp = (ramp_function_source_t*) user_malloc(sizeof(ramp_function_source_t));
  memset(ramp, 0, sizeof(ramp_function_source_t));

  ramp->frame_period = config->frame_period;
  ramp->max_value = config->max_value;
  ramp->min_value = config->min_value;
  ramp->increase_value = config->increase_value;
  ramp->decrease_value = config->decrease_value;
  ramp->ramp_state = config->ramp_state;
  ramp->target = 0.0f;
  ramp->out = 0.0f;

  return ramp;
}

/**
 * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
 * @author         GUATAI
 * @param[in]      斜波函数结构体
 * @param[in]      输入值
 * @retval         返回空
 */
//TODO:(GUATAI)可以考虑利用定时器来进行控制器目标值更新
float ramp_calc(ramp_function_source_t *ramp, float input)
{
  ramp->target = input;
  if (ramp->ramp_state == SLOPE_FIRST_REAL)
  {
    if ((ramp->target >= ramp->real_value && ramp->real_value >= ramp->plan_value) || (ramp->target <= ramp->real_value && ramp->real_value <= ramp->plan_value))
    {
      ramp->out = ramp->real_value;
    }
    if (ramp->plan_value > 0.0f)
    {
      //正值加速
      if (ramp->target > ramp->plan_value)
      {
	if (user_abs((ramp->plan_value - ramp->target)) > ramp->increase_value)
	{
	  ramp->out += ramp->increase_value;
	}
	else
	{
	  ramp->out = ramp->target;
	}
      }
      //正值减速
      else if (ramp->target < ramp->plan_value)
      {
	if (user_abs((ramp->plan_value - ramp->target)) > ramp->decrease_value)
	{
	  ramp->out -= ramp->decrease_value;
	}
	else
	{
	  ramp->out = ramp->target;
	}
      }
    }
    else if (ramp->plan_value < 0.0f)
    {
      //负值加速
      if (ramp->target < ramp->plan_value)
      {
	if (user_abs((ramp->plan_value - ramp->target)) > ramp->increase_value)
	{
	  ramp->out -= ramp->increase_value;
	}
	else
	{
	  ramp->out = ramp->target;
	}
      }
      //负值减速
      else if (ramp->target > ramp->plan_value)
      {
	if (user_abs((ramp->plan_value - ramp->target)) > ramp->decrease_value)
	{
	  ramp->out += ramp->decrease_value;
	}
	else
	{
	  ramp->out = ramp->target;
	}
      }
    }
    else
    {
      if (ramp->target > ramp->plan_value)
      {
	//0值正加速
	if (user_abs((ramp->plan_value - ramp->target)) > ramp->increase_value)
	{
	  ramp->out += ramp->increase_value;
	}
	else
	{
	  ramp->out = ramp->target;
	}
      }
      else if (ramp->target < ramp->plan_value)
      {
	//0值负加速
	if (user_abs((ramp->plan_value - ramp->target)) > ramp->increase_value)
	{
	  ramp->out -= ramp->increase_value;
	}
	else
	{
	  ramp->out = ramp->target;
	}
      }
    }
    ramp->plan_value = ramp->out;
  }
  else if (ramp->ramp_state == SLOPE_FIRST_TARGET)
  {
    ramp->out += ramp->target * ramp->frame_period;
    if (ramp->out > ramp->max_value)
    {
      ramp->out = ramp->max_value;
    }
    else if (ramp->out < ramp->min_value)
    {
      ramp->out = ramp->min_value;
    }
  }
  return ramp->out;
}

void* zmalloc(size_t size)
{
  void *ptr = malloc(size);
  memset(ptr, 0, size);
  return ptr;
}

// 快速开方
float Sqrt(float x)
{
  float y;
  float delta;
  float maxError;

  if (x <= 0)
  {
    return 0;
  }

  // initial guess
  y = x / 2;

  // refine
  maxError = x * 0.001f;

  do
  {
    delta = (y * y) - x;
    y -= delta / (2 * y);
  }
  while (delta > maxError || delta < -maxError);

  return y;
}

// 绝对值限制
float abs_limit(float num, float Limit)
{
  if (num > Limit)
  {
    num = Limit;
  }
  else if (num < -Limit)
  {
    num = -Limit;
  }
  return num;
}

// 判断符号位
float sign(float value)
{
  if (value >= 0.0f)
  {
    return 1.0f;
  }
  else
  {
    return -1.0f;
  }
}

// 浮点死区
float float_deadband(float Value, float minValue, float maxValue)
{
  if (Value < maxValue && Value > minValue)
  {
    Value = 0.0f;
  }
  return Value;
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue)
{
  if (maxValue < minValue)
  {
    return Input;
  }

  if (Input > maxValue)
  {
    float len = maxValue - minValue;
    while (Input > maxValue)
    {
      Input -= len;
    }
  }
  else if (Input < minValue)
  {
    float len = maxValue - minValue;
    while (Input < minValue)
    {
      Input += len;
    }
  }
  return Input;
}

// 弧度格式化为-PI~PI

// 角度格式化为-180~180
float theta_format(float Ang)
{
  return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
  static int integer;
  static float decimal;
  integer = (int) raw;
  decimal = raw - integer;
  if (decimal > 0.5f)
    integer++;
  return integer;
}

// 三维向量归一化
float* Norm3d(float *v)
{
  float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v[0] /= len;
  v[1] /= len;
  v[2] /= len;
  return v;
}

// 计算模长
float NormOf3d(float *v)
{
  return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 三维向量叉乘v1 x v2
void Cross3d(float *v1, float *v2, float *res)
{
  res[0] = v1[1] * v2[2] - v1[2] * v2[1];
  res[1] = v1[2] * v2[0] - v1[0] * v2[2];
  res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float *v1, float *v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

void MatInit(mat *m, uint8_t row, uint8_t col)
{
  m->numCols = col;
  m->numRows = row;
  m->pData = (float*) zmalloc(row * col * sizeof(float));
}

/**
 * @brief          普通最小二乘法初始化
 * @param[in]      普通最小二乘法结构体
 * @param[in]      样本数
 * @retval         返回空
 */
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order)
{
  OLS->Order = order;
  OLS->Count = 0;
  OLS->x = (float*) user_malloc(sizeof(float) * order);
  OLS->y = (float*) user_malloc(sizeof(float) * order);
  OLS->k = 0;
  OLS->b = 0;
  memset((void*) OLS->x, 0, sizeof(float) * order);
  memset((void*) OLS->y, 0, sizeof(float) * order);
  memset((void*) OLS->t, 0, sizeof(float) * 4);
}

/**
 * @brief          普通最小二乘法拟合
 * @param[in]      普通最小二乘法结构体
 * @param[in]      信号新样本距上一个样本时间间隔
 * @param[in]      信号值
 */
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
  static float temp = 0;
  temp = OLS->x[1];
  for (uint16_t i = 0; i < OLS->Order - 1; ++i)
  {
    OLS->x[i] = OLS->x[i + 1] - temp;
    OLS->y[i] = OLS->y[i + 1];
  }
  OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
  OLS->y[OLS->Order - 1] = y;

  if (OLS->Count < OLS->Order)
  {
    OLS->Count++;
  }
  memset((void*) OLS->t, 0, sizeof(float) * 4);
  for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
  {
    OLS->t[0] += OLS->x[i] * OLS->x[i];
    OLS->t[1] += OLS->x[i];
    OLS->t[2] += OLS->x[i] * OLS->y[i];
    OLS->t[3] += OLS->y[i];
  }

  OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
  OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

  OLS->StandardDeviation = 0;
  for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
  {
    OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
  }
  OLS->StandardDeviation /= OLS->Order;
}

/**
 * @brief          普通最小二乘法提取信号微分
 * @param[in]      普通最小二乘法结构体
 * @param[in]      信号新样本距上一个样本时间间隔
 * @param[in]      信号值
 * @retval         返回斜率k
 */
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
  static float temp = 0;
  temp = OLS->x[1];
  for (uint16_t i = 0; i < OLS->Order - 1; ++i)
  {
    OLS->x[i] = OLS->x[i + 1] - temp;
    OLS->y[i] = OLS->y[i + 1];
  }
  OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
  OLS->y[OLS->Order - 1] = y;

  if (OLS->Count < OLS->Order)
  {
    OLS->Count++;
  }

  memset((void*) OLS->t, 0, sizeof(float) * 4);
  for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
  {
    OLS->t[0] += OLS->x[i] * OLS->x[i];
    OLS->t[1] += OLS->x[i];
    OLS->t[2] += OLS->x[i] * OLS->y[i];
    OLS->t[3] += OLS->y[i];
  }

  OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

  OLS->StandardDeviation = 0;
  for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
  {
    OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
  }
  OLS->StandardDeviation /= OLS->Order;

  return OLS->k;
}

/**
 * @brief          获取普通最小二乘法提取信号微分
 * @param[in]      普通最小二乘法结构体
 * @retval         返回斜率k
 */
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS)
{
  return OLS->k;
}

/**
 * @brief          普通最小二乘法平滑信号
 * @param[in]      普通最小二乘法结构体
 * @param[in]      信号新样本距上一个样本时间间隔
 * @param[in]      信号值
 * @retval         返回平滑输出
 */
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y)
{
  static float temp = 0;
  temp = OLS->x[1];
  for (uint16_t i = 0; i < OLS->Order - 1; ++i)
  {
    OLS->x[i] = OLS->x[i + 1] - temp;
    OLS->y[i] = OLS->y[i + 1];
  }
  OLS->x[OLS->Order - 1] = OLS->x[OLS->Order - 2] + deltax;
  OLS->y[OLS->Order - 1] = y;

  if (OLS->Count < OLS->Order)
  {
    OLS->Count++;
  }

  memset((void*) OLS->t, 0, sizeof(float) * 4);
  for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
  {
    OLS->t[0] += OLS->x[i] * OLS->x[i];
    OLS->t[1] += OLS->x[i];
    OLS->t[2] += OLS->x[i] * OLS->y[i];
    OLS->t[3] += OLS->y[i];
  }

  OLS->k = (OLS->t[2] * OLS->Order - OLS->t[1] * OLS->t[3]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);
  OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / (OLS->t[0] * OLS->Order - OLS->t[1] * OLS->t[1]);

  OLS->StandardDeviation = 0;
  for (uint16_t i = OLS->Order - OLS->Count; i < OLS->Order; ++i)
  {
    OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
  }
  OLS->StandardDeviation /= OLS->Order;

  return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

/**
 * @brief          获取普通最小二乘法平滑信号
 * @param[in]      普通最小二乘法结构体
 * @retval         返回平滑输出
 */
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS)
{
  return OLS->k * OLS->x[OLS->Order - 1] + OLS->b;
}

//判断是否出现NAN
bool Judge_IF_NAN(float x )
{
  bool res = 0;
  res = (bool)__ARM_isnan((double)x);
  return res;
}