/**
* @file shoot.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __SHOOT_H__
#define __SHOOT_H__

#include <stdint.h>
#include "robot_frame_init.h"

#define SHOOT_MODE_FIRE 2
#define SHOOT_MODE_READY 1
#define SHOOT_MODE_STOP 0

typedef struct 
{
    /* data */
}__attribute__((packed))shoot_behaviour_t;

typedef struct
{
    /* data */
}__attribute__((packed))shoot_cmd_t;    

extern uint8_t shoot_mode;
extern uint8_t shoot_mode_last;

void Shoot_Init(void);
void Shoot_Enable(void);
void Shoot_Stop(void);
void Get_Shoot_Mode(void);
void Shoot_State_Machine(void);

#endif /* __SHOOT_H__ */