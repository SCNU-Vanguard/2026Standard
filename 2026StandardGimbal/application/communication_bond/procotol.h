/**
* @file procotol.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __PROCOTOL_H__ 
#define __PROCOTOL_H__

#include <stdint.h>

typedef struct
{
    /* data */
}__attribute__((packed))procotol_behaviour_t;

typedef struct
{
    /* data */
}__attribute__((packed))procotol_cmd_t;

void VOFA_Display_IMU(void);
void VOFA_Display_Pitch(void);

#endif /* __PROCOTOL_H__ */