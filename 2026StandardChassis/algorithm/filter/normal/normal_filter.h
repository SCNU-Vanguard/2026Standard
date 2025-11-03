/**
 * @file normal_filter.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __NORMAL_FILTER_H__
#define __NORMAL_FILTER_H__

#include <stdint.h>

/*-------------------------普通滤波器---------------------------*/
/*-----------------------------------------------------------*/
#define MAF_MAXSIZE 100
#define MIDF_MAXSIZE 100
#define MAF_ANTI_TOP_MAXSIZE 1000

//滑动平均滤波器
typedef struct
{
  float num[MAF_MAXSIZE];
  uint8_t lenth;
  uint8_t pot; //当前位置
  float total_value;
  float aver_value;

} moving_average_filter_t;	//最大设置MAF_MaxSize个

//滑动平均Pro滤波器
typedef struct
{
  float num[MAF_ANTI_TOP_MAXSIZE];
  uint16_t lenth;
  uint16_t pot;	//当前位置
  float total_value;
  float aver_value;

  float max;
  float min;
} maf_anti_top_t;	//最大设置MAF_MaxSize个

//索引数组负责记录数据进入数据窗口时pot是第几号
//提供删除数据的索引
//中值滤波器
typedef struct
{
  float data_num[MIDF_MAXSIZE];	//数值数组
  int data_index_num[MIDF_MAXSIZE];
  uint8_t lenth;

  uint8_t index_pot;	//始终指向下一个要删除的pot
  float median_data;

} median_filter_t;	//最大设置MAF_MaxSize个

extern moving_average_filter_t KEY_W, KEY_A, KEY_S, KEY_D;
extern moving_average_filter_t MOUSE_X, MOUSE_Y;

extern maf_anti_top_t Absolute_yaw_angle_raw, Absolute_pitch_angle_raw, Absolute_distance_raw;

//滑动滤波器对应的操作函数
void Average_Add(moving_average_filter_t *Aver, float add_data);
float Average_Get(moving_average_filter_t *Aver, uint16_t pre);	//获取前n次的数据
void Average_Init(moving_average_filter_t *Aver, uint8_t lenth);
void Average_Clear(moving_average_filter_t *Aver);
void Average_Fill(moving_average_filter_t *Aver, float temp);	//往滑动滤波填充某个值

//反向最大值滤波器
void Maf_Anti_Top_Add(maf_anti_top_t *Aver, float add_data);
float Maf_Anti_Top_Get(maf_anti_top_t *Aver, uint16_t pre);	//获取前n次的数据
void Maf_Anti_Top_Init(maf_anti_top_t *Aver, uint16_t lenth);
void Maf_Anti_Top_Clear(maf_anti_top_t *Aver);

//中值滤波器对应的操作函数
void Median_Add(median_filter_t *Median, float add_data);
float Median_Get(median_filter_t *Median, uint16_t pre);	//获取前n次的数据
void Median_Init(median_filter_t *Median, uint8_t lenth);
void Median_Clear(median_filter_t *Median);

#endif /* __NORMAL_FILTER_H__ */