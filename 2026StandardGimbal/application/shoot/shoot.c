/**
******************************************************************************
* @file    shoot.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>

#include "shoot.h"
#include "gimbal.h"
#include "remote_control.h"
#include "rs485.h"

uint8_t shoot_mode = 0;
uint8_t shoot_mode_last = 0;

void Shoot_Init(void)
{

}

void Shoot_Enable(void)
{
   
}

void Shoot_Stop(void)
{
    
}

void Get_Shoot_Mode(void)
{
    if (switch_is_up(rc_data->rc.switch_right))
    {
        shoot_mode = SHOOT_MODE_FIRE;
    }
    else if (switch_is_mid(rc_data->rc.switch_right))
    {
        shoot_mode = SHOOT_MODE_READY;
    }
    else if (switch_is_down(rc_data->rc.switch_right))
    {
        shoot_mode = SHOOT_MODE_STOP;
    }
    else
    {
        shoot_mode = SHOOT_MODE_STOP;
    }
}

void Shoot_State_Machine(void)
{
    static uint16_t init_count = 0;
    if (init_count < 1000)
    {
        init_count++;
        shoot_mode = SHOOT_MODE_STOP;
    }
    if (gimbal_mode)
    {
        switch (shoot_mode)
        {
        case SHOOT_MODE_FIRE:
            uart2_tx_message.shoot_mode = 1;
            break;

        case SHOOT_MODE_READY:
            uart2_tx_message.shoot_mode = 0;
            break;

        case SHOOT_MODE_STOP:
            uart2_tx_message.shoot_mode = 0;
            break;

        default:
            uart2_tx_message.shoot_mode = 0;
            break;
        }
    }
    else
    {
        uart2_tx_message.shoot_mode = 0;
    }
}