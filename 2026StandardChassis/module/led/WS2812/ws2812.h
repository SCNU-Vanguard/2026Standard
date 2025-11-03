/**
* @file ws2812.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __WS2812_H__
#define __WS2812_H__

#include <stdint.h>
#include "bsp_spi.h"

#define RED_WS2812_COLOR 0xff0000
#define ORANGE_WS2812_COLOR 0xffa308
#define YELLOW_WS2812_COLOR 0xffee46
#define WHITE_WS2812_COLOR 0x444444
#define BLUE_WS2812_COLOR 0x2b74ce
#define PINK_WS2812_COLOR 0xc586b6
#define GREEN_WS2812_COLOR 0x40e476

typedef struct
{
	SPI_instance_t *ws2812_spi;
} ws2812_instance_t;

typedef struct
{
	spi_init_config_t ws2812_spi_config;
} ws2812_init_config_t;

extern ws2812_init_config_t ws2812_config;
extern ws2812_instance_t *ws2812_instance;

ws2812_instance_t *WS2812_Register(ws2812_init_config_t *config);

void WS2812_Control_Three(ws2812_instance_t *ws2812, uint8_t r, uint8_t g, uint8_t b);

void WS2812_Control(ws2812_instance_t *ws2812, uint32_t color);

#endif /* __WS2812_H__ */
