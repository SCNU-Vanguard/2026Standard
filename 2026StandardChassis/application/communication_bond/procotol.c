/**
******************************************************************************
 * @file    procotol.c
 * @brief
 * @author
 ******************************************************************************
 * Copyright (c) 2023 Team
 * All rights reserved.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

#include "procotol.h"
#include "robot_frame_init.h"

#include "vofa.h"
#include "bmi088.h"
#include "super_cap.h"
#include "PowerCtrl.h"
#include "referee_task.h"
extern bmi088_data_t imu_data;

typedef enum
{
	SUPER_CAP_POLICY_ERROR_FALLBACK = 0,
	SUPER_CAP_POLICY_LOW_MINUS_30,
	SUPER_CAP_POLICY_HOLD,
	SUPER_CAP_POLICY_MINUS_15,
	SUPER_CAP_POLICY_PLUS_15,
	SUPER_CAP_POLICY_PLUS_30,
} super_cap_power_policy_e;

static super_cap_power_policy_e super_cap_power_policy = SUPER_CAP_POLICY_ERROR_FALLBACK;
float super_cap_chassis_power_target;

static uint16_t SuperCap_Clamp_U16(uint16_t value, uint16_t min, uint16_t max)
{
	if (value < min)
	{
		return min;
	}
	if (value > max)
	{
		return max;
	}
	return value;
}

static float SuperCap_Clamp_Float(float value, float min, float max)
{
	if (value < min)
	{
		return min;
	}
	if (value > max)
	{
		return max;
	}
	return value;
}


void SuperCap_PowerControl_Update(void)
{
	uint16_t referee_power_limit = SUPER_CAP_POWER_INIT;
	uint16_t referee_energy_buffer = SUPER_CAP_ENERGY_BUFFER_INIT;
	uint8_t cap_energy;
	uint8_t cap_unavailable;
	float target_power;

	if (Super_Cap_instance == NULL)
	{
		chassis_max_power = SUPER_CAP_CHASSIS_POWER_MIN_W;
		super_cap_power_policy = SUPER_CAP_POLICY_ERROR_FALLBACK;
		return;
	}

	if (referee_outer_info != NULL)
	{
		referee_power_limit = referee_outer_info->RobotPerformance.chassis_power_limit;
		referee_energy_buffer = referee_outer_info->PowerHeatData.buffer_energy;
	}

	referee_power_limit = SuperCap_Clamp_U16(referee_power_limit,
											SUPER_CAP_POWER_LIMIT_MIN_W,
											SUPER_CAP_POWER_LIMIT_MAX_W);
	referee_energy_buffer = SuperCap_Clamp_U16(referee_energy_buffer,
											  0U,
											  SUPER_CAP_ENERGY_BUFFER_INIT);

	cap_energy = Super_Cap_instance->receive_data.capEnergy;
	cap_unavailable = (Super_Cap_instance->online == 0U) ||
					  (Super_Cap_instance->receive_data.errorCode != SUPER_CAP_ERROR_NONE);

	if (cap_unavailable)
	{
		target_power = (float)referee_power_limit;
		Power_Control_Mode = CODE_CONTROL;
		super_cap_power_policy = SUPER_CAP_POLICY_ERROR_FALLBACK;
	}
	else if (cap_energy < SUPER_CAP_ENERGY_15_RAW)
	{
		target_power = (float)referee_power_limit * 0.70f;
		Power_Control_Mode = CODE_CONTROL;
		super_cap_power_policy = SUPER_CAP_POLICY_LOW_MINUS_30;
	}
	else if (cap_energy <= SUPER_CAP_ENERGY_20_RAW)
	{
		target_power = chassis_max_power; // 15%~20%保持上一档，避免边界反复跳变
		super_cap_power_policy = SUPER_CAP_POLICY_HOLD;
	}
	else if (cap_energy < SUPER_CAP_ENERGY_40_RAW)
	{
		target_power = (float)referee_power_limit * 0.85f;
		Power_Control_Mode = CODE_CONTROL;
		super_cap_power_policy = SUPER_CAP_POLICY_MINUS_15;
	}
	else if (cap_energy < SUPER_CAP_ENERGY_70_RAW)
	{
		target_power = (float)referee_power_limit * 1.15f;
		Power_Control_Mode = SUPERCAP_CONTROL;
		super_cap_power_policy = SUPER_CAP_POLICY_PLUS_15;
	}
	else
	{
		target_power = (float)referee_power_limit * 1.30f;
		Power_Control_Mode = SUPERCAP_CONTROL;
		super_cap_power_policy = SUPER_CAP_POLICY_PLUS_30;
	}

	super_cap_chassis_power_target = SuperCap_Clamp_Float(target_power,
														 SUPER_CAP_CHASSIS_POWER_MIN_W,
														 SUPER_CAP_CHASSIS_POWER_MAX_W);
	chassis_max_power = super_cap_chassis_power_target;

	Super_Cap_Enable(Super_Cap_instance);
	Super_Cap_instance->transmit_data.refereePowerLimit = referee_power_limit;
	Super_Cap_instance->transmit_data.refereeEnergyBuffer = referee_energy_buffer;
	Super_Cap_SendData(Super_Cap_instance);
}