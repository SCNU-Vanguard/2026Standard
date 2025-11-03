/**
******************************************************************************
 * @file    ws2812.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */
#include <stdlib.h>
#include <string.h>
#include "ws2812.h"

#define WS2812_LowLevel    0xC0     // 0
#define WS2812_HighLevel   0xF0     // 1

ws2812_init_config_t ws2812_config;
ws2812_instance_t *ws2812_instance;

ws2812_instance_t *WS2812_Register(ws2812_init_config_t *config)
{
	ws2812_instance_t *instance = (ws2812_instance_t *) malloc(sizeof(ws2812_instance_t));
	memset(instance, 0, sizeof(ws2812_instance_t));

	config->ws2812_spi_config.id = instance;

	config->ws2812_spi_config.spi_work_mode = SPI_BLOCK_MODE;
	config->ws2812_spi_config.callback      = NULL;

	config->ws2812_spi_config.spi_handle = &hspi6;
	config->ws2812_spi_config.GPIOx      = NULL;
	config->ws2812_spi_config.cs_pin     = NULL;
	config->ws2812_spi_config.cs_flag    = 0;

	instance->ws2812_spi = SPI_Register(&config->ws2812_spi_config);

	return instance;
}

void WS2812_Control_Three(ws2812_instance_t *ws2812, uint8_t r, uint8_t g, uint8_t b)
{
	uint8_t txbuf[24];
	uint8_t res = 0;
	for (int i = 0 ; i < 8 ; i++)
	{
		txbuf[7 - i]  = (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
		txbuf[15 - i] = (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
		txbuf[23 - i] = (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
	}
	//	SPI_Transmit(ws2812->ws2812_spi, &res, 0);
	//    while (ws2812->ws2812_spi->spi_handle->State != HAL_SPI_STATE_READY);
	SPI_Transmit(ws2812->ws2812_spi, txbuf, 24);
	for (int i = 0 ; i < 100 ; i++)
	{
		SPI_Transmit(ws2812->ws2812_spi, &res, 1);
	}
}

//(0xff0000)// 红色
//(0xffa308)// 橙色
//(0xffee46)// 黄色
//(0x444444)// 白色
//(0x43c9b0)// 青色
//(0x2b74ce)// 蓝色
//(0xc586b6)// 粉色
//(0x40e476)// 鲜绿
//(0x111111)//
//(0x000000)//灭
void WS2812_Control(ws2812_instance_t *ws2812, uint32_t color)
{
	static uint8_t txbuf[24];

	for (int i = 0; i < 8; i++)
	{
		txbuf[7-i]  = (((color>>(i+8))&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
		txbuf[15-i] = (((color>>(i+16))&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
		txbuf[23-i] = (((color>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
	}

  SPI_Transmit(ws2812->ws2812_spi, txbuf, 24);
}