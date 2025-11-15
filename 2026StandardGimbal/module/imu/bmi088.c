#include <stdlib.h>
#include <string.h>
#include "bmi088_reg.h"
#include "bmi088.h"
#include "buzzer.h"

#include "bsp_dwt.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "user_lib.h"

#define BMI088_BSP_COM 1
#define BMI088_AUTO_CS 1
#define BMI088_EXTI 0

#define BMI088_Cali 0

bmi088_init_config_t bmi088_init_h7 = {
	.heat_pid_config = {
		.kp = 80.0f,
		.ki = 0.5f,
		.kd = 0,
		.integral_limit = 200.0f,
		.output_limit = 2000.0f,
	},
	.heat_pwm_config = {
		.htim = &htim3,
		.channel = TIM_CHANNEL_4,
		.dutyratio = 0,
		.period = 5000 - 1,
	},

	.spi_acc_config = {
		.GPIOx = GPIOC,
		.cs_pin = GPIO_PIN_0,
		.spi_handle = &hspi2,
#if BMI088_AUTO_CS
		.cs_flag = SPI_CS_AUTO,
#else
		.cs_flag = SPI_CS_EXTERN,
#endif
	},
	.spi_gyro_config = {
		.GPIOx = GPIOC,
		.cs_pin = GPIO_PIN_3,
		.spi_handle = &hspi2,
#if BMI088_AUTO_CS
		.cs_flag = SPI_CS_AUTO,
#else
		.cs_flag = SPI_CS_EXTERN,
#endif
	},

	.acc_int_config = {
		.GPIOx = GPIOE,
		.GPIO_Pin = GPIO_PIN_10,
		.exti_mode = GPIO_EXTI_MODE_RISING,
		.gpio_model_callback = NULL,
	},
	.gyro_int_config = {
		.GPIOx = GPIOE,
		.GPIO_Pin = GPIO_PIN_12,
		.exti_mode = GPIO_EXTI_MODE_RISING,
		.gpio_model_callback = NULL,
	},

#if BMI088_Cali
	.cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
#else
	.cali_mode = BMI088_LOAD_PRE_CALI_MODE,
#endif
#if BMI088_EXTI
	.work_mode = BMI088_BLOCK_TRIGGER_MODE,
#else
	.work_mode = BMI088_BLOCK_PERIODIC_MODE,
#endif
};

bmi088_instance_t *bmi088_h7  = NULL;
SPI_HandleTypeDef *BMI088_SPI = &hspi2;

// ---------------------------原始数据滤波选择--------------------------------//
#define BMI088_USE_KALMAN_FILTER 1

#if BMI088_USE_KALMAN_FILTER

#include "kalman_one_filter.h"
float test_data_acc[3]  = {0};
float test_data_gyro[3] = {0};
kalman_one_filter_t acc_kf[3];
kalman_one_filter_t gyro_kf[3];

static void BMI088_Kalman_Filter_Init(void)
{
	for(uint8_t i = 0 ; i < 3 ; i++)
	{
		Kalman_One_Init(&acc_kf[i], 0.01f, 200.0f);  // 加速度卡尔曼滤波器初始化
		Kalman_One_Init(&gyro_kf[i], 0.01f, 20.0f); // 陀螺仪卡尔曼滤波器初始化
	}
}

#endif

// ---------------------------原始数据滤波选择--------------------------------//

// ---------------------------以下私有函数,用于读写BMI088寄存器封装,blocking--------------------------------//
#if BMI088_BSP_COM
#else
static uint8_t BMI088_Read_Write(uint8_t tx_data)
{
	uint8_t rx_data = 0;
	HAL_SPI_TransmitReceive(BMI088_SPI, &tx_data, &rx_data, 1, 1000);
	return rx_data;
}
#endif

/**
 * @brief 读取BMI088寄存器Accel. BMI088要求在不释放CS的情况下连续读取
 *
 * @param bmi088 待读取的BMI088实例
 * @param reg 待读取的寄存器地址
 * @param dataptr 读取到的数据存放的指针
 * @param len 读取长度
 */
static void BMI088_Accel_Read(bmi088_instance_t *bmi088, uint8_t reg, uint8_t *dataptr, uint8_t len)
{
#if BMI088_BSP_COM
	if (len > 6)
	{
		while (1);
	}

#if BMI088_AUTO_CS
#else
	HAL_GPIO_WritePin(bmi088->spi_acc->GPIOx, bmi088->spi_acc->cs_pin, GPIO_PIN_RESET);
#endif

	// 一次读取最多6个字节,加上两个dummy data    第一个字节的第一个位是读写位,1为读,0为写,1-7bit是寄存器地址
	static uint8_t tx[8] = {0x80, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}; // 读取,第一个字节为0x80|reg ,第二个是dummy data,后面的没用都是dummy write
	static uint8_t rx[8] = {0}; // 前两个字节是dummy data,第三个开始是真正的数据
	tx[0]                = 0x80 | reg;   // 静态变量每次进来还是上次的值,所以要每次都要给tx[0]赋值0x80
	SPI_Transmit_Receive(bmi088->spi_acc, rx, tx, len + 2);

#if BMI088_AUTO_CS
#else
	HAL_GPIO_WritePin(bmi088->spi_acc->GPIOx, bmi088->spi_acc->cs_pin, GPIO_PIN_SET);
#endif

	memcpy(dataptr, rx + 2, len); // @todo : memcpy有额外开销,后续可以考虑优化,在SPI中加入接口或模式,使得在一次传输结束后不释放CS,直接接着传输
#else
	if (len == 1)
	{



#if BMI088_AUTO_CS
#else
	HAL_GPIO_WritePin(bmi088->spi_acc->GPIOx, bmi088->spi_acc->cs_pin, GPIO_PIN_RESET);
#endif

	BMI088_Read_Write(0x80 | reg);
	BMI088_Read_Write(0x55);
	*dataptr = BMI088_Read_Write(0x55);

#if BMI088_AUTO_CS
#else
	HAL_GPIO_WritePin(bmi088->spi_acc->GPIOx, bmi088->spi_acc->cs_pin, GPIO_PIN_SET);
#endif
	}
	else
	{



#if BMI088_AUTO_CS
#else
	HAL_GPIO_WritePin(bmi088->spi_acc->GPIOx, bmi088->spi_acc->cs_pin, GPIO_PIN_RESET);
#endif

	BMI088_Read_Write(0x80 | reg);
	BMI088_Read_Write(0x80 | reg);

	while (len != 0)
	{
		*dataptr = BMI088_Read_Write(0x55);
		dataptr++;
		len--;
	}

#if BMI088_AUTO_CS
#else
	HAL_GPIO_WritePin(bmi088->spi_acc->GPIOx, bmi088->spi_acc->cs_pin, GPIO_PIN_SET);
#endif
	}
#endif
}

/**
 * @brief 读取BMI088寄存器Gyro, BMI088要求在不释放CS的情况下连续读取
 *
 * @param bmi088 待读取的BMI088实例
 * @param reg  待读取的寄存器地址
 * @param dataptr 读取到的数据存放的指针
 * @param len 读取长度
 */
static void BMI088_Gyro_Read(bmi088_instance_t *bmi088, uint8_t reg, uint8_t *dataptr, uint8_t len)
{
#if BMI088_BSP_COM
	if (len > 6)
	{
		while (1);
	}

#if BMI088_AUTO_CS
#else
	HAL_GPIO_WritePin(bmi088->spi_gyro->GPIOx, bmi088->spi_gyro->cs_pin, GPIO_PIN_RESET);
#endif

	// 一次读取最多6个字节,加上一个dummy data  ,第一个字节的第一个位是读写位,1为读,0为写,1-7bit是寄存器地址
	static uint8_t tx[8] = {0x80, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}; // 读取,第一个字节为0x80 | reg ,之后是dummy data
	static uint8_t rx[8] = {0};          // 第一个是dummy data,第三个开始是真正的数据
	tx[0]                = 0x80 | reg;
	SPI_Transmit_Receive(bmi088->spi_gyro, rx, tx, len + 1);

#if BMI088_AUTO_CS
#else
	HAL_GPIO_WritePin(bmi088->spi_gyro->GPIOx, bmi088->spi_gyro->cs_pin, GPIO_PIN_SET);
#endif

	memcpy(dataptr, rx + 1, len); // @todo : memcpy有额外开销,后续可以考虑优化,在SPI中加入接口或模式,使得在一次传输结束后不释放CS,直接接着传输
#else
	if (len == 1)
	{



#if BMI088_AUTO_CS
#else
	HAL_GPIO_WritePin(bmi088->spi_gyro->GPIOx, bmi088->spi_gyro->cs_pin, GPIO_PIN_RESET);
#endif

	BMI088_Read_Write(0x80 | reg);
	*dataptr = BMI088_Read_Write(0x55);

#if BMI088_AUTO_CS
#else
	HAL_GPIO_WritePin(bmi088->spi_gyro->GPIOx, bmi088->spi_gyro->cs_pin, GPIO_PIN_SET);
#endif
	}
	else
	{



#if BMI088_AUTO_CS
#else
	HAL_GPIO_WritePin(bmi088->spi_gyro->GPIOx, bmi088->spi_gyro->cs_pin, GPIO_PIN_RESET);
#endif

	BMI088_Read_Write(0x80 | reg);

	while (len != 0)
	{
		*dataptr = BMI088_Read_Write(0x55);
		dataptr++;
		len--;
	}

#if BMI088_AUTO_CS
#else
	HAL_GPIO_WritePin(bmi088->spi_gyro->GPIOx, bmi088->spi_gyro->cs_pin, GPIO_PIN_SET);
#endif
	}
#endif
}

/**
 * @brief 写accel寄存器.对spitransmit形式上的封装
 * @attention 只会向目标reg写入一个字节,因为只有1个字节所以直接传值(指针是32位反而浪费)
 *
 * @param bmi088 待写入的BMI088实例
 * @param reg  待写入的寄存器地址
 * @param data 待写入的数据(注意不是指针)
 */
static void BMI088_Accel_Write_SingleReg(bmi088_instance_t *bmi088, uint8_t reg, uint8_t data)
{
	uint8_t tx[2] = {reg, data};
	SPI_Transmit(bmi088->spi_acc, tx, 2);
}

/**
 * @brief 写gyro寄存器.形式上的封装
 * @attention 只会向目标reg写入一个字节,因为只有1个字节所以直接传值(指针是32位反而浪费)
 *
 * @param bmi088 待写入的BMI088实例
 * @param reg  待写入的寄存器地址
 * @param data 待写入的数据(注意不是指针)
 */
static void BMI088_Gyro_Write_SingleReg(bmi088_instance_t *bmi088, uint8_t reg, uint8_t data)
{
	uint8_t tx[2] = {reg, data};
	SPI_Transmit(bmi088->spi_gyro, tx, 2);
}

// -------------------------以上为私有函数,封装了BMI088寄存器读写函数,blocking--------------------------------//

// -------------------------以下为私有函数,用于初始化BMI088acc和gyro的硬件和配置--------------------------------//
#define BMI088REG   0
#define BMI088DATA  1
#define BMI088ERROR 2

// BMI088初始化配置数组for accel,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
static uint8_t bmi088_accel_init_table[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
	{BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
	{BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
	{BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
	{BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
	{BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
	{BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}};

// BMI088初始化配置数组for gyro,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
static uint8_t bmi088_gyro_init_table[BMI088_WRITE_GYRO_REG_NUM][3] =
{
	{BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
	{BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
	{BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
	{BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
	{BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
	{BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};
// @attention : 以上两个数组配合各自的初始化函数使用. 若要修改请参照BMI088 datasheet

/**
 * @brief 初始化BMI088加速度计,提高可读性分拆功能
 *
 * @param bmi088 待初始化的BMI088实例
 * @return uint8_t BMI088ERROR CODE if any problems here
 */
static uint8_t BMI088_Accel_Init(bmi088_instance_t *bmi088)
{
	uint8_t WhoAmI_check = 0;

	BMI088_Accel_Read(bmi088, BMI088_ACC_CHIP_ID, &WhoAmI_check, 1); // dummy read,
	// HAL_Delay(1);
	DWT_Delay(0.001f);
	BMI088_Accel_Read(bmi088, BMI088_ACC_CHIP_ID, &WhoAmI_check, 1); // dummy read,
	// HAL_Delay(1);
	DWT_Delay(0.001f);

	BMI088_Accel_Write_SingleReg(bmi088, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE); // 软复位
	// HAL_Delay(80);
	DWT_Delay(0.08f);

	BMI088_Accel_Read(bmi088, BMI088_ACC_CHIP_ID, &WhoAmI_check, 1); // dummy read,
	// HAL_Delay(1);
	DWT_Delay(0.001f);
	BMI088_Accel_Read(bmi088, BMI088_ACC_CHIP_ID, &WhoAmI_check, 1); // dummy read,
	// HAL_Delay(1);
	DWT_Delay(0.001f);

	// 检查ID,如果不是0x1E(bmi088 whoami寄存器值),则返回错误
	//	BMI088_Accel_Read(bmi088, BMI088_ACC_CHIP_ID, &WhoAmI_check, 1);
	if (WhoAmI_check != BMI088_ACC_CHIP_ID_VALUE)
		return BMI088_NO_SENSOR;
	//	DWT_Delay(0.001f);
	// 初始化寄存器,提高可读性
	uint8_t reg               = 0, data = 0;
	BMI088_ERORR_CODE_e error = 0;
	data                      = sizeof(bmi088_accel_init_table) / sizeof(bmi088_accel_init_table[0]);
	// 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
	for (uint8_t i = 0 ; i < sizeof(bmi088_accel_init_table) / sizeof(bmi088_accel_init_table[0]) ; i++)
	{
		reg  = bmi088_accel_init_table[i][BMI088REG];
		data = bmi088_accel_init_table[i][BMI088DATA];
		BMI088_Accel_Write_SingleReg(bmi088, reg, data); // 写入寄存器
		//HAL_Delay(1);
		DWT_Delay(0.001f);

		BMI088_Accel_Read(bmi088, reg, &data, 1); // 写完之后立刻读回检查
		//HAL_Delay(1);
		DWT_Delay(0.001f);

		if (data != bmi088_accel_init_table[i][BMI088DATA])
			error |= bmi088_accel_init_table[i][BMI088ERROR];
		//{i--;} 可以设置retry次数,如果retry次数用完了,则返回error
	}
	// 设置灵敏度
	switch (bmi088_accel_init_table[3][1])
	{
		case BMI088_ACC_RANGE_3G:
			bmi088->BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
			break;
		case BMI088_ACC_RANGE_6G:
			bmi088->BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
			break;
		case BMI088_ACC_RANGE_12G:
			bmi088->BMI088_ACCEL_SEN = BMI088_ACCEL_12G_SEN;
			break;
		case BMI088_ACC_RANGE_24G:
			bmi088->BMI088_ACCEL_SEN = BMI088_ACCEL_24G_SEN;
			break;
		default:
			break;
	}
	return (uint8_t) error;
}

/**
 * @brief 初始化BMI088陀螺仪,提高可读性分拆功能
 *
 * @param bmi088 待初始化的BMI088实例
 * @return uint8_t BMI088ERROR CODE
 */
static uint8_t BMI088_Gyro_Init(bmi088_instance_t *bmi088)
{
	uint8_t WhoAmI_check = 0;

	BMI088_Gyro_Read(bmi088, BMI088_GYRO_CHIP_ID, &WhoAmI_check, 1); // dummy read,
	// HAL_Delay(1);
	DWT_Delay(0.001f);
	BMI088_Gyro_Read(bmi088, BMI088_GYRO_CHIP_ID, &WhoAmI_check, 1); // dummy read,
	// HAL_Delay(1);
	DWT_Delay(0.001f);

	// 后续添加reset和通信检查?
	// code to go here ...
	BMI088_Gyro_Write_SingleReg(bmi088, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE); // 软复位
	// HAL_Delay(80);
	DWT_Delay(0.08f);

	BMI088_Gyro_Read(bmi088, BMI088_GYRO_CHIP_ID, &WhoAmI_check, 1); // dummy read,
	// HAL_Delay(1);
	DWT_Delay(0.001f);
	BMI088_Gyro_Read(bmi088, BMI088_GYRO_CHIP_ID, &WhoAmI_check, 1); // dummy read,
	// HAL_Delay(1);
	DWT_Delay(0.001f);

	// 检查ID,如果不是0x0F(bmi088 whoami寄存器值),则返回错误
	//	BMI088_Gyro_Read(bmi088, BMI088_GYRO_CHIP_ID, &WhoAmI_check, 1);
	if (WhoAmI_check != BMI088_GYRO_CHIP_ID_VALUE)
		return BMI088_NO_SENSOR;
	//	DWT_Delay(0.001f);

	// 初始化寄存器,提高可读性
	uint8_t reg               = 0, data = 0;
	BMI088_ERORR_CODE_e error = 0;
	// 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
	for (uint8_t i = 0 ; i < sizeof(bmi088_gyro_init_table) / sizeof(bmi088_gyro_init_table[0]) ; i++)
	{
		reg  = bmi088_gyro_init_table[i][BMI088REG];
		data = bmi088_gyro_init_table[i][BMI088DATA];
		BMI088_Gyro_Write_SingleReg(bmi088, reg, data); // 写入寄存器
		//HAL_Delay(1);
		DWT_Delay(0.001f);
		BMI088_Gyro_Read(bmi088, reg, &data, 1); // 写完之后立刻读回对应寄存器检查是否写入成功
		//HAL_Delay(1);
		DWT_Delay(0.001f);
		if (data != bmi088_gyro_init_table[i][BMI088DATA])
			error |= bmi088_gyro_init_table[i][BMI088ERROR];
		//{i--;} 可以设置retry次数,尝试重新写入.如果retry次数用完了,则返回error
	}
	// 设置灵敏度
	switch (bmi088_gyro_init_table[0][1])
	{
		case BMI088_GYRO_2000:
			bmi088->BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
			break;
		case BMI088_GYRO_1000:
			bmi088->BMI088_GYRO_SEN = BMI088_GYRO_1000_SEN;
			break;
		case BMI088_GYRO_500:
			bmi088->BMI088_GYRO_SEN = BMI088_GYRO_500_SEN;
			break;
		case BMI088_GYRO_250:
			bmi088->BMI088_GYRO_SEN = BMI088_GYRO_250_SEN;
			break;
		case BMI088_GYRO_125:
			bmi088->BMI088_GYRO_SEN = BMI088_GYRO_125_SEN;
			break;
		default:
			break;
	}
	return (uint8_t) error;
}

// -------------------------以上为私有函数,用于初始化BMI088acc和gyro的硬件和配置--------------------------------//

/**
 * @brief          控制bmi088的温度
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
void BMI088_Temp_Control(bmi088_instance_t *bmi088)
{
	static uint8_t temp_constant_time = 0;
	static uint8_t first_temperate    = 0; // 第一次达到设定温度
	static float target_temp          = 0;
	target_temp                       = bmi088->ambient_temperature + 10; // 推荐比环境温度高10度
	if (target_temp > 45.0f)
		target_temp = 45.0f;                         // 限制在45度以内

	if (first_temperate)
	{
		PID_Position(bmi088->heat_pid, target_temp, bmi088->temperature);
		// 限制在正数范围
		if (bmi088->heat_pid->output < 0.0f)
		{
			bmi088->heat_pid->output = 0.0f;
		}
		if (bmi088->heat_pid->i_out < 0)
		{
			bmi088->heat_pid->i_out = 0;
		}
		PWM_Set_DutyRatio(bmi088->heat_pwm, bmi088->heat_pid->output);
	}
	else
	{
		// 在没有达到设置的温度-4，一直最大功率加热
		PWM_Set_DutyRatio(bmi088->heat_pwm, 0.95f);
		if (bmi088->temperature > target_temp - 4)
		{
			temp_constant_time++;
			if (temp_constant_time > 200)
			{
				// 达到设置温度，设置积分项，加速收敛
				first_temperate         = 1;
				bmi088->heat_pid->i_out = 0.05f;
			}
		}
	}
}

// -------------------------以下为私有函数,private用于IT模式下的中断处理---------------------------------//

static void BMI088_Accel_SPI_Finish_Callback(SPI_instance_t *spi)
{
	static bmi088_instance_t *bmi088;
	static uint16_t callback_time = 0;
	bmi088                        = (bmi088_instance_t *) (spi->id);
	// 如果是加速度计的中断,则启动加速度计数据读取,并转换为实际值
	if (bmi088->update_flag.acc == 1)
	{
		for (uint8_t i   = 0 ; i < 3 ; i++)
			bmi088->acc[i] = bmi088->BMI088_ACCEL_SEN * (float) (int16_t) (((bmi088->acc_raw[2 * i + 1]) << 8) | bmi088->acc_raw[2 * i]);
		bmi088->update_flag.acc = 0;
	}

	if (callback_time >= 500)
	{
		BMI088_Accel_Read(bmi088, BMI088_TEMP_M, bmi088->temp_raw, 2);
		int16_t temperate_raw_temp;
		temperate_raw_temp = (int16_t) ((bmi088->temp_raw[0] << 3) | (bmi088->temp_raw[1] >> 5));
		if (temperate_raw_temp > 1023)
			temperate_raw_temp -= 2048;

		bmi088->temperature = temperate_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
		// 设置环境温度
		if (bmi088->ambient_temperature < 0)
		{
			bmi088->ambient_temperature = bmi088->temperature;
		}
		BMI088_Temp_Control(bmi088);
		callback_time = 0;
	}
	callback_time++;
}

extern osThreadId_t INS_task_handel; // 本来是全局变量直接extern

static void BMI088_Gyro_SPI_Finish_Callback(SPI_instance_t *spi)
{
	static bmi088_instance_t *bmi088 = NULL;
	bmi088                           = (bmi088_instance_t *) (spi->id);
	// 将陀螺仪数据转换为实际值
	for (uint8_t i    = 0 ; i < 3 ; i++)
		bmi088->gyro[i] = bmi088->BMI088_GYRO_SEN * (float) (int16_t) (((bmi088->gyro_raw[2 * i + 1]) << 8) | bmi088->gyro_raw[2 * i]);
	bmi088->update_flag.gyro = 0;
	// 由于SPI速率为10.5Mbit/s，超出了BMI088的10Mbit/s限制，所以有些陀螺仪数据会出错
	// 有尝试调低速率，但是这样加速度计中断在开机几秒后便无法触发，原因未知
	///@todo 找到原因降低速率，使数据恢复正常，可能原因是初始化需要较低的速率，之后可以拉满
	// 下面的函数用于过滤错误数据,无法完全过滤
	// 寄完了

	// begin
	for (uint8_t i = 0 ; i < 3 ; i++)
	{
		if (fabsf(bmi088->last_gyro[i] - bmi088->gyro[i]) > 10.0f)
			return;
	}
	for (uint8_t i = 0 ; i < 3 ; i++)
	{
		bmi088->last_gyro[i] = bmi088->gyro[i];
	}
	// end
	bmi088->update_flag.imu_ready = 1;
	osThreadFlagsSet(INS_task_handel, IMU_READY_FLAG); // 通知主线程IMU数据准备完毕（以陀螺仪中断为准 1000Hz）
}

static void BMI088_Accel_INT_Callback(GPIO_instance_t *gpio)
{
	static bmi088_instance_t *bmi088;
	bmi088 = (bmi088_instance_t *) (gpio->id);
	// 启动加速度计数据读取,并转换为实际值
	BMI088_Accel_Read(bmi088, BMI088_ACCEL_XOUT_L, bmi088->acc_raw, 6);
	bmi088->update_flag.acc = 1;
	// 读取完毕会调用BMI088AccSPIFinishCallback
}

static void BMI088_Gyro_INT_Callback(GPIO_instance_t *gpio)
{
	static bmi088_instance_t *bmi088 = NULL;
	bmi088                           = (bmi088_instance_t *) (gpio->id);
	// 启动陀螺仪数据读取,并转换为实际值
	BMI088_Gyro_Read(bmi088, BMI088_GYRO_X_L, bmi088->gyro_raw, 6);
	bmi088->update_flag.gyro = 1;
	// 读取完毕会调用BMI088GyroSPIFinishCallback
}

// -------------------------以上为私有函数,private用于IT模式下的中断处理---------------------------------//

// -------------------------以下为私有函数,用于改变BMI088的配置--------------------------------//
static void BMI088_Set_Mode(bmi088_instance_t *bmi088Instance, bmi088_work_mode_e mode)
{
	bmi088Instance->work_mode = mode;
	if (mode == BMI088_BLOCK_PERIODIC_MODE)
	{
		SPI_Set_Mode(bmi088Instance->spi_acc, SPI_BLOCK_MODE);
		SPI_Set_Mode(bmi088Instance->spi_gyro, SPI_BLOCK_MODE);
	}
	else if (mode == BMI088_BLOCK_TRIGGER_MODE)
	{
		SPI_Set_Mode(bmi088Instance->spi_acc, SPI_DMA_MODE);
		SPI_Set_Mode(bmi088Instance->spi_gyro, SPI_DMA_MODE);
	}
}

// -------------------------以上为私有函数,用于改变BMI088的配置--------------------------------//

// -------------------------以下为公有函数,用于注册BMI088,标定和数据读取--------------------------------//

/**
 * @brief
 * @param bmi088
 * @return bmi088_data_t
 */
uint8_t BMI088_Read_All(bmi088_instance_t *bmi088, bmi088_data_t *data_store)
{
	// 如果是blocking模式,则主动触发一次读取并返回数据
	if (bmi088->work_mode == BMI088_BLOCK_PERIODIC_MODE)
	{
		static uint8_t buf[6] = {0}; // 最多读取6个byte(gyro/acc,temp是2)
		int16_t raw_temp;
		// 读取accel的x轴数据首地址,bmi088内部自增读取地址 // 3* sizeof(int16_t)
		BMI088_Accel_Read(bmi088, BMI088_ACCEL_XOUT_L, buf, 6);
		for (uint8_t i = 0 ; i < 3 ; i++)
		{
			raw_temp = (float) (int16_t) (((buf[2 * i + 1]) << 8) | buf[2 * i]);
#if BMI088_USE_KALMAN_FILTER				
				data_store->acc[i] = raw_temp;
#else
			if (bmi088->cali_mode == BMI088_CALIBRATE_ONLINE_MODE)
			{
				data_store->acc[i] = bmi088->BMI088_ACCEL_SEN * raw_temp;				
			}
			else
			{
				data_store->acc[i] = (bmi088->BMI088_ACCEL_SEN * raw_temp * bmi088->accel_scale - bmi088->acc_offset[i]) * 0.75f + bmi088->acc[i] * 0.25f;				
			}
#endif			
		}

		BMI088_Gyro_Read(bmi088, BMI088_GYRO_X_L, buf, 6); // 连续读取3个(3*2=6)轴的角速度
		for (uint8_t i = 0 ; i < 3 ; i++)
		{
			raw_temp = (float) (int16_t) (((buf[2 * i + 1]) << 8) | buf[2 * i]);
#if BMI088_USE_KALMAN_FILTER				
			data_store->gyro[i] = raw_temp;
#else
			if (bmi088->cali_mode == BMI088_CALIBRATE_ONLINE_MODE)
			{
				data_store->gyro[i] = bmi088->BMI088_GYRO_SEN * raw_temp;				
			}
			else
			{
				data_store->gyro[i] = (bmi088->BMI088_GYRO_SEN * raw_temp) - bmi088->gyro_offset[i];			
			}
#endif			
		}

		BMI088_Accel_Read(bmi088, BMI088_TEMP_M, buf, 2); // 读温度,温度传感器在accel上

		data_store->temperature = (float) ((int16_t) ((buf[0] << 3) | (buf[1] >> 5))) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
		// 更新BMI088自身结构体数据
		for (uint8_t i = 0 ; i < 3 ; i++)
		{
#if BMI088_USE_KALMAN_FILTER
//			bmi088->acc[i] = data_store->acc[i];
//			bmi088->gyro[i] = data_store->gyro[i];
//			test_data_acc[i]  = Kalman_One_Filter(&acc_kf[i], data_store->acc[i]);
//			test_data_gyro[i] = Kalman_One_Filter(&gyro_kf[i], data_store->gyro[i]);
			bmi088->acc[i]  = Kalman_One_Filter(&acc_kf[i], data_store->acc[i]);
			bmi088->gyro[i] = Kalman_One_Filter(&gyro_kf[i], data_store->gyro[i]);
			if (bmi088->cali_mode == BMI088_CALIBRATE_ONLINE_MODE)
			{
				bmi088->acc[i] = bmi088->BMI088_ACCEL_SEN * bmi088->acc[i];
				bmi088->gyro[i] = bmi088->BMI088_GYRO_SEN * bmi088->gyro[i];
			}
			else
			{
				bmi088->acc[i] = bmi088->BMI088_ACCEL_SEN * bmi088->acc[i] * bmi088->accel_scale - bmi088->acc_offset[i];
				bmi088->gyro[i] = bmi088->BMI088_GYRO_SEN * bmi088->gyro[i] - bmi088->gyro_offset[i];
			}				
			data_store->acc[i] = bmi088->acc[i];
			data_store->gyro[i] = bmi088->gyro[i];
#else
			bmi088->acc[i]  = data_store->acc[i];
			bmi088->gyro[i] = data_store->gyro[i];
#endif			
		}

		bmi088->temperature = data_store->temperature;
		return 1;
	}
	// 如果是IT模式,则检查标志位.当传感器数据准备好会触发外部中断,中断服务函数会将标志位置1
	if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE && bmi088->update_flag.imu_ready == 1)
	{
		data_store->acc[0]            = bmi088->acc[0];
		data_store->acc[1]            = bmi088->acc[1];
		data_store->acc[2]            = bmi088->acc[2];
		data_store->gyro[0]           = bmi088->gyro[0];
		data_store->gyro[1]           = bmi088->gyro[1];
		data_store->gyro[2]           = bmi088->gyro[2];
		data_store->temperature       = bmi088->temperature;
		bmi088->update_flag.imu_ready = 0;
		return 1;
	}
	// 如果数据还没准备好,则返回空数据
	if (bmi088->update_flag.imu_ready == 0)
	{
		data_store = NULL;
		return 0;
	}
	return 255;
}

/**
 * @brief :  读取BMI088的IMU更新完成标志位
 * @param *bmi088
 * @return 1 数据准备完毕 0 没有数据
 */
uint8_t BMI088_Acquire_IT_Status(bmi088_instance_t *bmi088)
{
	// 只有中断才能读取标志位
	if (bmi088->work_mode == BMI088_BLOCK_TRIGGER_MODE && bmi088->update_flag.imu_ready == 1)
	{
		bmi088->update_flag.imu_ready = 0;
		return 1;
	}
	else
		return 0;
}

/**
 * @brief BMI088 gyro 标定
 * @attention 不管工作模式是blocking还是IT,标定时都是blocking模式,所以不用担心中断关闭后无法标定(RobotInit关闭了全局中断)
 * @param _bmi088 待标定的BMI088实例
 */

void BMI088_Calibrate_IMU(bmi088_instance_t *_bmi088)
{
//	bmi088_work_mode_e mode = _bmi088->work_mode;
//	BMI088_Set_Mode(_bmi088, BMI088_BLOCK_PERIODIC_MODE);
	
	if (_bmi088->cali_mode == BMI088_CALIBRATE_ONLINE_MODE) // 性感bmi088在线标定
	{
		static float gyro_diff[3], g_norm_diff;
		
		static float start_time       = 0.0f;
		static uint16_t cail_time     = 6000;
		static uint16_t acc_cali_time = 6000;
		int16_t bmi088_raw_temp;
		float gyro_max[3], gyro_min[3];
		float g_norm_temp, g_norm_max, g_norm_min;

		static uint16_t cali_time_count = 0;

				uint8_t WhoAmI_check = 0;		

				BMI088_Accel_Read(_bmi088, BMI088_ACC_CHIP_ID, &WhoAmI_check, 1); // dummy read,
				// HAL_Delay(1);
				DWT_Delay(0.001f);
				BMI088_Accel_Read(_bmi088, BMI088_ACC_CHIP_ID, &WhoAmI_check, 1); // dummy read,
				// HAL_Delay(1);
				DWT_Delay(0.001f);

				if (WhoAmI_check != BMI088_ACC_CHIP_ID_VALUE)
					while(1){;}
				
				BMI088_Gyro_Read(_bmi088, BMI088_GYRO_CHIP_ID, &WhoAmI_check, 1); // dummy read,
				// HAL_Delay(1);
				DWT_Delay(0.001f);
				BMI088_Gyro_Read(_bmi088, BMI088_GYRO_CHIP_ID, &WhoAmI_check, 1); // dummy read,
				// HAL_Delay(1);
				DWT_Delay(0.001f);
				
				//	BMI088_Gyro_Read(bmi088, BMI088_GYRO_CHIP_ID, &WhoAmI_check, 1);
				if (WhoAmI_check != BMI088_GYRO_CHIP_ID_VALUE)
					while(1){;}

		start_time = DWT_GetTimeline_s( );
		do
		{
			if (DWT_GetTimeline_s( ) - start_time > 12.01f)
			{
				_bmi088->gyro_offset[0] = GYRO_X_OFFSET;
				_bmi088->gyro_offset[1] = GYRO_Y_OFFSET;
				_bmi088->gyro_offset[2] = GYRO_Z_OFFSET;
				_bmi088->acc_offset[0]  = ACCEL_X_OFFSET;
				_bmi088->acc_offset[1]  = ACCEL_Y_OFFSET;
				_bmi088->acc_offset[2]  = ACCEL_Z_OFFSET;
				_bmi088->g_norm         = G_NORM;
				_bmi088->temperature    = 40.0F; // 40度
				break;
			}

			DWT_Delay(0.0005f);
			_bmi088->g_norm = 0;

			for (uint8_t i = 0 ; i < 3 ; i++)
			{
				_bmi088->gyro_offset[i] = 0.0f;
			}

			bmi088_data_t bmi088_data = {0};
			for (uint16_t i = 0 ; i < cail_time ; i++)
			{
				//读imu数据(转化后)
				BMI088_Read_All(_bmi088, &bmi088_data);

				g_norm_temp = NormOf3d(bmi088_data.acc);

				_bmi088->g_norm += g_norm_temp;

				for (uint8_t j = 0 ; j < 3 ; j++)
				{
					_bmi088->gyro_offset[j] += bmi088_data.gyro[j]; // 逐渐减小偏移量
				}

				if (i == 0)
				{
					g_norm_max = g_norm_temp;
					g_norm_min = g_norm_temp;

					for (uint8_t j = 0 ; j < 3 ; j++)
					{
						gyro_max[j] = bmi088_data.gyro[j];
						gyro_min[j] = bmi088_data.gyro[j];
					}
				}
				else
				{
					if (g_norm_temp > g_norm_max)
					{
						g_norm_max = g_norm_temp;
					}
					if (g_norm_temp < g_norm_min)
					{
						g_norm_min = g_norm_temp;
					}

					for (uint8_t j = 0 ; j < 3 ; j++)
					{
						if (bmi088_data.gyro[j] > gyro_max[j])
						{
							gyro_max[j] = bmi088_data.gyro[j];
						}
						if (bmi088_data.gyro[j] < gyro_min[j])
						{
							gyro_min[j] = bmi088_data.gyro[j];
						}
					}
				}

				// 数据差异过大认为收到外界干扰，需重新校准
				g_norm_diff = g_norm_max - g_norm_min;
				for (uint8_t j = 0 ; j < 3 ; j++)
				{
					gyro_diff[j] = gyro_max[j] - gyro_min[j];
				}
				if (g_norm_diff > 0.5f || gyro_diff[0] > 0.15f || gyro_diff[1] > 0.15f || gyro_diff[2] > 0.15f)
				{
					break;
				}

				DWT_Delay(0.0005f);

				if (cali_time_count % 1000 == 0)
				{
					Buzzer_One_Note(1047, 0.2f, 1);
				}
				cali_time_count++;
			}

			_bmi088->g_norm /= (float) cail_time; // 计算平均值

			for (uint8_t i = 0 ; i < 3 ; i++)
			{
				_bmi088->gyro_offset[i] /= (float) cail_time; // 计算偏移量
			}

			_bmi088->cali_temperature = _bmi088->temperature;
		} while (g_norm_diff > 0.5f ||
		         fabsf(_bmi088->g_norm - 9.8f) > 0.5f ||
		         gyro_diff[0] > 0.15f ||
		         gyro_diff[1] > 0.15f ||
		         gyro_diff[2] > 0.15f ||
		         fabsf(_bmi088->gyro_offset[0]) > 0.01f ||
		         fabsf(_bmi088->gyro_offset[1]) > 0.01f ||
		         fabsf(_bmi088->gyro_offset[2]) > 0.01f);

		_bmi088->accel_scale = 9.81f / _bmi088->g_norm; // 计算加速度计的缩放系数

		for (uint16_t i = 0 ; i < acc_cali_time ; i++)
		{
			bmi088_data_t bmi088_data = {0};
			BMI088_Read_All(_bmi088, &bmi088_data);
			for (uint8_t j = 0 ; j < 3 ; j++)
			{
				if (j == 2)
				{
					_bmi088->acc_offset[j] = bmi088_data.acc[j] * _bmi088->accel_scale; // 逐渐减小偏移量
				}
				else
				{
					_bmi088->acc_offset[j] = bmi088_data.acc[j] * _bmi088->accel_scale * 0.25f + _bmi088->acc_offset[j] * 0.75f; // 逐渐减小偏移量
				}
			}

			DWT_Delay(0.0005f);

			if (cali_time_count % 1000 == 0)
			{
				Buzzer_One_Note(1047, 0.2f, 1);
			}
			cali_time_count++;
		}
	}
	// 导入数据
	else if (_bmi088->cali_mode == BMI088_LOAD_PRE_CALI_MODE)
	{
		_bmi088->gyro_offset[0]   = GYRO_X_OFFSET;
		_bmi088->gyro_offset[1]   = GYRO_Y_OFFSET;
		_bmi088->gyro_offset[2]   = GYRO_Z_OFFSET;
		_bmi088->acc_offset[0]    = ACCEL_X_OFFSET;
		_bmi088->acc_offset[1]    = ACCEL_Y_OFFSET;
		_bmi088->acc_offset[2]    = ACCEL_Z_OFFSET;
		_bmi088->g_norm           = G_NORM;
		_bmi088->cali_temperature = 40.0F; // 40度
		_bmi088->accel_scale      = 9.81f / _bmi088->g_norm;
	}

	_bmi088->acc_offset[2]    = 0.0f;
	_bmi088->cali_mode = BMI088_LOAD_PRE_CALI_MODE;
//	BMI088_Set_Mode(_bmi088, mode); // 恢复工作模式
}

bmi088_instance_t *BMI088_Register(bmi088_init_config_t *config)
{
	// 申请内存
	bmi088_instance_t *bmi088_instance = (bmi088_instance_t *) malloc(sizeof(bmi088_instance_t));
	memset(bmi088_instance, 0, sizeof(bmi088_instance_t)); // 清零
	// 从右向左赋值,让bsp instance保存指向bmi088_instance的指针(父指针),便于在底层中断中访问bmi088_instance
	config->acc_int_config.id      =
			config->gyro_int_config.id =
			config->spi_acc_config.id  =
			config->spi_gyro_config.id =
			config->heat_pwm_config.id = bmi088_instance;

	// 根据参数选择工作模式
	if (config->work_mode == BMI088_BLOCK_PERIODIC_MODE)
	{
		config->spi_acc_config.spi_work_mode  = SPI_BLOCK_MODE;
		config->spi_gyro_config.spi_work_mode = SPI_BLOCK_MODE;
		// callbacks are all NULL
	}
	else if (config->work_mode == BMI088_BLOCK_TRIGGER_MODE)
	{
		config->spi_gyro_config.spi_work_mode = SPI_DMA_MODE; // 如果DMA资源不够,可以用SPI_IT_MODE
		config->spi_gyro_config.spi_work_mode = SPI_DMA_MODE;
		// 设置回调函数
		config->spi_acc_config.callback             = BMI088_Accel_SPI_Finish_Callback;
		config->spi_gyro_config.callback            = BMI088_Gyro_SPI_Finish_Callback;
		config->acc_int_config.gpio_model_callback  = BMI088_Accel_INT_Callback;
		config->gyro_int_config.gpio_model_callback = BMI088_Gyro_INT_Callback;
		bmi088_instance->acc_int                    = GPIO_Register(&config->acc_int_config); // 只有在非阻塞模式下才需要注册中断
		bmi088_instance->gyro_int                   = GPIO_Register(&config->gyro_int_config);
	}
	// 注册实例
	bmi088_instance->spi_acc  = SPI_Register(&config->spi_acc_config);
	bmi088_instance->spi_gyro = SPI_Register(&config->spi_gyro_config);
	bmi088_instance->heat_pwm = PWM_Register(&config->heat_pwm_config);

	bmi088_instance->heat_pid = PID_Init(&config->heat_pid_config);

	bmi088_instance->ambient_temperature = -273; // 环境温度初值
	DWT_GetDeltaT(&bmi088_instance->bias_dwt_cnt);
	// 初始化时使用阻塞模式
	BMI088_Set_Mode(bmi088_instance, BMI088_BLOCK_PERIODIC_MODE);
	// 初始化acc和gyro
	BMI088_ERORR_CODE_e error = BMI088_NO_ERROR;
	do
	{
		static uint8_t error_count = 0;
		error                      = BMI088_NO_ERROR;
		error |= BMI088_Accel_Init(bmi088_instance);
		error |= BMI088_Gyro_Init(bmi088_instance);
		error_count++;
		if (error_count == 10)
		{
			Buzzer_One_Note(7902, 0.2f, 1); // 10次初始化失败报警
			break;
		}
	} while (error != 0);
#if BMI088_USE_KALMAN_FILTER
	BMI088_Kalman_Filter_Init();
#endif
//	bmi088_instance->cali_mode = BMI088_LOAD_PRE_CALI_MODE;
//	BMI088_Calibrate_IMU(bmi088_instance);               // 标定acc和gyro
	bmi088_instance->cali_mode = config->cali_mode;
	BMI088_Calibrate_IMU(bmi088_instance);               // 标定acc和gyro
	BMI088_Set_Mode(bmi088_instance, config->work_mode); // 恢复工作模式

	return bmi088_instance;
}
