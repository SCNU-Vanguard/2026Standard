/**
* @file chassis.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include <stdint.h>
#include "DJI_motor.h"
#include "DM_motor.h"

typedef struct 
{
    /* data */
}__attribute__((packed))chassis_behaviour_t;

typedef struct
{
    /* data */
}__attribute__((packed))chassis_cmd_t;

// extern DJI_motor_instance_t *DJI_motor_init_3508_test_instance1;
// extern DJI_motor_instance_t *DJI_motor_init_3508_test_instance2;
// extern DJI_motor_instance_t *DJI_motor_init_3508_test_instance3;
// extern DJI_motor_instance_t *DJI_motor_init_3508_test_instance4;

// extern DJI_motor_instance_t *DJI_motor_init_6020_test_instance1;
// extern DJI_motor_instance_t *DJI_motor_init_6020_test_instance2;
// extern DJI_motor_instance_t *DJI_motor_init_6020_test_instance3;
// extern DJI_motor_instance_t *DJI_motor_init_6020_test_instance4;

// extern DM_motor_t *DM_motor_init_4310_test_instance1;

void Chassis_Init(void);

#endif /* __CHASSIS_H__ */