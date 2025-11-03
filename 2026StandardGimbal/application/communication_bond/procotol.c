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

#include "vofa.h"
#include "bmi088.h"

extern bmi088_data_t imu_data;

void VOFA_Display_IMU(void)
{
    vofa_data_view[0] = imu_data.acc[0];
    vofa_data_view[1] = imu_data.acc[1];
    vofa_data_view[2] = imu_data.acc[2];

    vofa_data_view[3] = imu_data.gyro[0];
    vofa_data_view[4] = imu_data.gyro[1];
    vofa_data_view[5] = imu_data.gyro[2];

    vofa_data_view[6] = imu_data.temperature;

    VOFA_Send_Data(vofa_data_view, 7);
    // VOFA_JustFloat(vofa_data_view, 7);
}

void VOFA_Display_Pitch(void)
{

}