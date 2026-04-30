/**
* @file robot_frame_init.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __ROBOT_FRAME_INIT_H__
#define __ROBOT_FRAME_INIT_H__

#include <stdint.h>
#include "remote_control.h"

#include "remote_vt03.h"

extern RC_ctrl_t *rc_data;
extern VT03_ctrl_t *vt03_data;
extern void Robot_Frame_Init(void);

#endif /* __ROBOT_FRAME_INIT_H__ */