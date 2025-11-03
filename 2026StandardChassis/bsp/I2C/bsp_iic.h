/**
 * @file bsp_iic.h
 * @author guatai (2508588132@qq.com)
 * @brief 
 * @version 0.1
 * @date 2025-08-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __BSP_IIC_H__
#define __BSP_IIC_H__

#ifdef __cplusplus
extern "C"
{

#endif

#include <stdint.h>
#include "i2c.h"

#define IIC_DEVICE_CNT 2   // 主控板可支持的iic数量
#define MX_IIC_SLAVE_CNT 8 // 最大从机数目,根据需要修改

/* i2c 工作模式枚举 */
typedef enum
{
	// 基本工作模式
	IIC_BLOCK_MODE = 0, // 阻塞模式
	IIC_IT_MODE,        // 中断模式
	IIC_DMA_MODE,       // DMA模式
} iic_work_mode_e;

/* I2C MEM工作模式枚举,这两种方法都是阻塞 */
typedef enum
{
	IIC_READ_MEM = 0, // 读取从机内部的寄存器or内存
	IIC_WRITE_MEM,    // 写入从机内部的寄存器or内存
} iic_mem_mode_e;

/* Seq传输工作模式枚举,注意HOLD_ON要在IT或DMA下使用 */
// 必须以IIC_RELEASE为最后一次传输,否则会导致总线占有权无法释放
typedef enum
{
	IIC_SEQ_RELEASE,    // 完成传输后释放总线占有权,这是默认的传输方式
	IIC_SEQ_HOLDON = 0, // 保持总线占有权不释放,只支持IT和DMA模式
} iic_seq_mode_e;

/* i2c实例 */
typedef struct iic_temp_s
{
	I2C_HandleTypeDef *handle; // i2c handle
	uint8_t dev_address;       // 暂时只支持7位地址(还有一位是读写位)

	iic_work_mode_e work_mode;             // 工作模式
	uint8_t *rx_buffer;                    // 接收缓冲区指针
	uint8_t rx_len;                        // 接收长度
	void (*callback)(struct iic_temp_s *); // 接收完成后的回调函数

	void *id; // 用于标识i2c instance
} IIC_t;

/* I2C 初始化结构体配置 */
typedef struct
{
	I2C_HandleTypeDef *handle;       // i2c handle
	uint8_t dev_address;             // 暂时只支持7位地址(还有一位是读写位),注意不需要左移
	iic_work_mode_e work_mode;       // 工作模式
	void (*callback)(IIC_t *); // 接收完成后的回调函数
	void *id;                        // 用于标识i2c instance
} iic_init_config_t;

/**
 * @brief IIC初始化
 *
 * @param conf 初始化配置
 * @return IIC_t*
 */
IIC_t *IIC_Register(iic_init_config_t *conf);

/**
 * @brief IIC设置工作模式
 *
 * @param iic 要设置的iic实例
 * @param mode 工作模式
 */
void IIC_Set_Mode(IIC_t *iic, iic_work_mode_e mode);

/**
 * @brief IIC发送数据
 *
 * @param iic iic实例
 * @param data 待发送的数据首地址指针
 * @param size 发送长度
 * @param mode 序列传输模式
 * @note 注意,如果发送结构体,那么该结构体在声明时务必使用#pragma pack(1)进行对齐,并在声明结束后使用#pragma pack()恢复对齐
 *
 */
void IIC_Transmit(IIC_t *iic, uint8_t *data, uint16_t size, iic_seq_mode_e mode);

/**
 * @brief IIC接收数据
 *
 * @param iic iic实例
 * @param data 接收数据的首地址指针
 * @param size 接收长度
 * @param mode 序列传输模式
 * @note 注意,如果直接将接收数据memcpy到目标结构体或通过强制类型转换进行逐字节写入,
 *       那么该结构体在声明时务必使用#pragma pack(1)进行对齐,并在声明结束后使用#pragma pack()恢复对齐
 */
void IIC_Receive(IIC_t *iic, uint8_t *data, uint16_t size, iic_seq_mode_e mode);

/**
 * @brief IIC读取从机寄存器(内存),只支持阻塞模式,超时默认为1ms
 *
 * @param iic iic实例
 * @param mem_addr 要读取的从机内存地址,目前只支持8位地址
 * @param data 要读取或写入的数据首地址指针
 * @param size 要读取或写入的数据长度
 * @param mode 写入或读取模式: IIC_READ_MEM or IIC_WRITE_MEM
 * @param mem8bit_flag 从机内存地址是否为8位
 */
void IIC_Access_Mem(IIC_t *iic, uint16_t mem_addr, uint8_t *data, uint16_t size, iic_mem_mode_e mode, uint8_t mem8bit_flag);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_IIC_H__ */
