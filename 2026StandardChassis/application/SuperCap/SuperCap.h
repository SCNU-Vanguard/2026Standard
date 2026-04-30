#ifndef __SUPERPOWER_H__
#define __SUPERPOWER_H__
#include "bsp_can.h"

#define POWER_LIMIT 75		//裁判限制功率 refree_data->Game_Robot_state.chassis_power_limit
#define ENERGY_BUFFER 60	//裁判能量缓冲 refree_data->Power_Heat_Data.buffer_energy

// can_init_config_t Chassis_power_can_init_config = {
//     .can_handle = &hfdcan2,
//     .tx_id = 0x061,
//     .rx_id = 0x051,
// };

typedef struct
{
    // float voltage;
    // float current;
    // float power;
    uint8_t StatusCode;
    float ChassisPower;
    uint16_t ChassisPowerLimit;
    uint8_t CapEnergy;
    CAN_instance_t *Chassis_power_can_instance;
} Chassis_Power_instance_t;

void Super_Power_Init(void);
static void Chassis_Power_Decode(CAN_instance_t *can_instance);
Chassis_Power_instance_t *Chassis_Power_Init(void);
void SuperCap_send_cmd(uint16_t power_limit, uint16_t RefereeEnergyBuffer);
void SuperCap_Enable(uint16_t power_limit, uint16_t RefereeEnergyBuffer);
void SuperCap_Disable(uint16_t power_limit, uint16_t RefereeEnergyBuffer);
void SuperCap_SystemRestart(uint16_t power_limit, uint16_t RefereeEnergyBuffer);
void SuperCap_ClearError(uint16_t power_limit, uint16_t RefereeEnergyBuffer);

extern Chassis_Power_instance_t *SuperCap_Data;

#endif /* __SPUERPOWER_H__ */