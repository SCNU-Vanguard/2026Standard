/**
******************************************************************************
* @file    chassis.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

// 4-------电池-------1
// |                  |
// |                  |
// |                  |
// |                  |
// |                  |
// 3------------------2

#include <string.h>
#include <stdlib.h>

#include "chassis.h"

#include "DM_motor.h"
#include "DJI_motor.h"

PID_t chassis_3508_speed_pid = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .output_limit = 10000.0f, 
    .integral_limit = 1000.0f,
    .dead_band = 0.0f,
};

PID_t chassis_6020_angle_pid = {
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = 400.0f,
    .output_limit = 4.0f, 
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
};

PID_t chassis_6020_speed_pid = {
    .kp = 1200.0f,
    .ki = 24.0f,
    .kd = 0.0f,
    .output_limit = 25000.0f, 
    .integral_limit = 25000.0f,
    .dead_band = 0.0f,
};

PID_t chassis_4310_angle_pid = {
    .kp = 30.0f,
    .ki = 0.0f,
    .kd = 600.0f,
    .output_limit = 5.0f, 
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
};

PID_t chassis_4310_speed_pid = {
    .kp = 1.0f,
    .ki = 0.002f,
    .kd = 0.0f,
    .output_limit = 10.0f, 
    .integral_limit = 10.0f,
    .dead_band = 0.0f,
};

motor_init_config_t chassis_3508_init_1 = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_3508_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = M3508,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x01,
        .rx_id = 0x01,
    },

    .motor_control_type = TORQUE_LOOP_CONTRO,
};

motor_init_config_t chassis_3508_init_2 = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_3508_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = M3508,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x02,
        .rx_id = 0x02,
    },

    .motor_control_type = TORQUE_LOOP_CONTRO,
};

motor_init_config_t chassis_3508_init_3 = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_3508_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = M3508,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x03,
        .rx_id = 0x03,
    },

    .motor_control_type = TORQUE_LOOP_CONTRO,
};

motor_init_config_t chassis_3508_init_4 = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_3508_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = M3508,

    .can_init_config = {
        .can_handle = &hfdcan1,
        .tx_id = 0x04,
        .rx_id = 0x04,
    },

    .motor_control_type = TORQUE_LOOP_CONTRO,
};

motor_init_config_t chassis_6020_init_1 = {
    .controller_param_init_config = {
        .angle_PID = &chassis_6020_angle_pid,
        .speed_PID = &chassis_6020_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_AND_SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = GM6020,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x01,
        .rx_id = 0x01,
    },
};

motor_init_config_t chassis_6020_init_2 = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_6020_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = GM6020,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x02,
        .rx_id = 0x02,
    },
};

motor_init_config_t chassis_6020_init_3 = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_6020_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = GM6020,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x03,
        .rx_id = 0x03,
    },
};

motor_init_config_t chassis_6020_init_4 = {
    .controller_param_init_config = {
        .angle_PID = NULL,
        .speed_PID = &chassis_6020_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = SPEED_LOOP,
        .close_loop_type = SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = GM6020,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x04,
        .rx_id = 0x04,
    },
};

motor_init_config_t chassis_4310_init = {
	.controller_param_init_config = {
		.angle_PID = &chassis_4310_angle_pid,
		.speed_PID = &chassis_4310_speed_pid,
		.current_PID = NULL,
		.torque_PID = NULL,

		.other_angle_feedback_ptr = NULL,
		.other_speed_feedback_ptr = NULL,

		.angle_feedforward_ptr = NULL,
		.speed_feedforward_ptr = NULL,
		.current_feedforward_ptr = NULL,
		.torque_feedforward_ptr = NULL,

		.pid_ref = 0.0f,
	},
	.controller_setting_init_config = {
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_AND_SPEED_LOOP,

		.motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
		.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,

		.feedforward_flag = FEEDFORWARD_NONE,
	},

	.motor_type = DM4310,

	.can_init_config = {
		.can_handle = &hfdcan3,
		.tx_id = 0x01,
		.rx_id = 0x11,
	},
};

DJI_motor_instance_t *chassis_motor_drive_1;
DJI_motor_instance_t *chassis_motor_drive_2;
DJI_motor_instance_t *chassis_motor_drive_3;
DJI_motor_instance_t *chassis_motor_drive_4;

DJI_motor_instance_t *chassis_motor_direct_1;
DJI_motor_instance_t *chassis_motor_direct_2;
DJI_motor_instance_t *chassis_motor_direct_3;
DJI_motor_instance_t *chassis_motor_direct_4;

// DM_motor_t *chassis_4310_instance;

void Chassis_Init(void)
{
    // chassis_motor_drive_1 = DJI_Motor_Init(&chassis_3508_init_1);
    // chassis_motor_drive_2 = DJI_Motor_Init(&chassis_3508_init_2);
    // chassis_motor_drive_3 = DJI_Motor_Init(&chassis_3508_init_3);
    // chassis_motor_drive_4 = DJI_Motor_Init(&chassis_3508_init_4);

    // chassis_motor_direct_1 = DJI_Motor_Init(&chassis_6020_init_1);
    // chassis_motor_direct_2 = DJI_Motor_Init(&chassis_6020_init_2);
    // chassis_motor_direct_3 = DJI_Motor_Init(&chassis_6020_init_3);
    // chassis_motor_direct_4 = DJI_Motor_Init(&chassis_6020_init_4);

    // chassis_4310_instance = DM_Motor_Init(&chassis_4310_init);   
    
}

void Chassis_Enable(void)
{
    DJI_Motor_Enable(chassis_motor_drive_1);
    DJI_Motor_Enable(chassis_motor_drive_2);
    DJI_Motor_Enable(chassis_motor_drive_3);
    DJI_Motor_Enable(chassis_motor_drive_4);

    DJI_Motor_Enable(chassis_motor_direct_1);
    DJI_Motor_Enable(chassis_motor_direct_2);
    DJI_Motor_Enable(chassis_motor_direct_3);
    DJI_Motor_Enable(chassis_motor_direct_4);
}

void Chassis_Stop(void)
{
    DJI_Motor_Stop(chassis_motor_drive_1);
    DJI_Motor_Stop(chassis_motor_drive_2);
    DJI_Motor_Stop(chassis_motor_drive_3);
    DJI_Motor_Stop(chassis_motor_drive_4);

    DJI_Motor_Stop(chassis_motor_direct_1);
    DJI_Motor_Stop(chassis_motor_direct_2);
    DJI_Motor_Stop(chassis_motor_direct_3);
    DJI_Motor_Stop(chassis_motor_direct_4);
}



