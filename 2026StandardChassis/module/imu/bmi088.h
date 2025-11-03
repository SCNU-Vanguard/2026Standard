/**
* @file bmi088.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __BMI088_H__ // 防止重复包含
#define __BMI088_H__

#include <stdint.h>
#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"

#include "pid.h"

#define IMU_READY_FLAG (1 << 0)

// 陀螺仪校准数据，开启陀螺仪校准后可从INS中获取
#define GYRO_X_OFFSET 0.00360078155f
#define GYRO_Y_OFFSET 0.00109721895f
#define GYRO_Z_OFFSET 0.00094577496f
#define ACCEL_X_OFFSET -0.02846324f
#define ACCEL_Y_OFFSET -0.08984510f
#define ACCEL_Z_OFFSET 0.0f
#define G_NORM 9.85204792f // 重力加速度
// 陀螺仪默认环境温度
#define BMI088_AMBIENT_TEMPERATURE 36.5f

// bmi088工作模式枚举
typedef enum
{                      //@note 阻塞是指解算是阻塞的，F4系列没有硬件DSP
	BMI088_BLOCK_PERIODIC_MODE = 0, // 阻塞模式,周期性读取
	BMI088_BLOCK_TRIGGER_MODE,      // 阻塞模式,触发读取(中断) //! 该模式会出现一些错误数据
} bmi088_work_mode_e;

// bmi088标定方式枚举,若使用预设标定参数,注意修改预设参数
typedef enum
{
	BMI088_CALIBRATE_ONLINE_MODE = 0, // 初始化时进行标定
	BMI088_LOAD_PRE_CALI_MODE,        // 使用预设标定参数,
} bmi088_calibrate_mode_e;

#pragma pack(1) // 按1字节对齐
/* BMI088数据*/
typedef struct
{
	float gyro[3];     // 陀螺仪数据,xyz
	float acc[3];      // 加速度计数据,xyz
	float temperature; // 温度
} bmi088_data_t;
#pragma pack()

/* BMI088实例结构体定义 */
typedef struct
{
	// 传输模式和工作模式控制
	bmi088_work_mode_e work_mode;
	bmi088_calibrate_mode_e cali_mode;
	// SPI接口
	SPI_instance_t *spi_gyro; // 注意,SPIInstnace内部也有一个GPIOInstance,用于控制片选CS
	SPI_instance_t *spi_acc;  // 注意,SPIInstnace内部也有一个GPIOInstance,用于控制片选CS
	// EXTI GPIO,如果BMI088工作在中断模式,则需要配置中断引脚(有数据产生时触发解算)
	GPIO_instance_t *gyro_int;
	GPIO_instance_t *acc_int;
	// 温度控制
	PID_t *heat_pid;     // 恒温PID
	PWM_instance_t *heat_pwm;     // 加热PWM
	float ambient_temperature; // 陀螺仪环境温度
	// RAW数据
	uint8_t gyro_raw[6];
	uint8_t acc_raw[6];
	uint8_t temp_raw[2];
	// IMU数据
	float gyro[3];     // 陀螺仪数据,
	float last_gyro[3]; // 上次陀螺仪数据,xyz
	float acc[3];      // 加速度计数据,xyz
	float temperature; // 温度
	float cali_temperature;
	// 标定数据
	float accel_scale;
	float gyro_offset[3]; // 陀螺仪零偏
	float acc_offset[3];  // 加速度计零偏
	float g_norm;
	// 传感器灵敏度,用于计算实际值(reg.h中定义)
	float BMI088_ACCEL_SEN;
	float BMI088_GYRO_SEN;
	// 用于计算两次采样的时间间隔
	uint32_t bias_dwt_cnt;

	// 数据更新标志位
	struct // 位域,节省空间提高可读性
	{
		uint8_t gyro        : 1; // 1:有新数据,0:无新数据
		uint8_t acc         : 1;
		uint8_t temp        : 1;
		uint8_t gyro_overrun: 1; // 1:数据溢出,0:无溢出
		uint8_t acc_overrun : 1;
		uint8_t temp_overrun: 1;
		uint8_t imu_ready   : 1; // 1:IMU数据准备好,0:IMU数据未准备好(gyro+acc)
		// 后续可添加其他标志位,不够用可以扩充16or32,太多可以删
	} update_flag;
} bmi088_instance_t;

/* BMI088初始化配置 */
typedef struct
{
	bmi088_work_mode_e work_mode;
	bmi088_calibrate_mode_e cali_mode;

	spi_init_config_t spi_gyro_config;
	spi_init_config_t spi_acc_config;

	gpio_init_config_t gyro_int_config;
	gpio_init_config_t acc_int_config;

	PID_t heat_pid_config;

	pwm_init_config_t heat_pwm_config;
} bmi088_init_config_t;

extern bmi088_init_config_t bmi088_init_h7;
extern bmi088_instance_t *bmi088_h7;

/**
 * @brief 初始化BMI088,返回BMI088实例指针
 * @note  一般一个开发板只有一个BMI088,所以这里就叫BMI088Init而不是Register
 *
 * @param config bmi088初始化配置
 * @return bmi088_instance_t* 实例指针
 */
bmi088_instance_t *BMI088_Register(bmi088_init_config_t *config);

/**
 * @brief 读取BMI088数据
 * @param bmi088 BMI088实例指针
 * @return bmi088_data_t 读取到的数据
 */
uint8_t BMI088_Read_All(bmi088_instance_t *bmi088, bmi088_data_t *data_store);

uint8_t BMI088_Acquire_IT_Status(bmi088_instance_t *bmi088);

void BMI088_Temp_Control(bmi088_instance_t *bmi088);

#endif /* __BMI088_H__ */
