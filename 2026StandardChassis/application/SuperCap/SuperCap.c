#include "DM_motor.h"
#include "bsp_dwt.h"
#include "SuperCap.h"
#include "chassis.h"

Chassis_Power_instance_t *SuperCap_Data;

void Super_Power_Init(void)
{
	SuperCap_Data = Chassis_Power_Init();
	SuperCap_Enable(POWER_LIMIT, ENERGY_BUFFER);//裁判限制功率&裁判能量缓冲
}

static void Chassis_Power_Decode(CAN_instance_t *can_instance)
{
    uint8_t *data = can_instance->rx_buff;
    uint8_t capEnergy = 0;
    uint8_t statusCode = 0;
    uint32_t power_uint32 = 0;
    float chassisPower = 0.0f;
    uint16_t chassisPowerLimit = 0;

    statusCode = data[0] & 0x03;//只要低两位
    
    power_uint32 |= (data[1] << 0);
    power_uint32 |= (data[2] << 8); 
    power_uint32 |= (data[3] << 16); 
    power_uint32 |= (data[4] << 24); 

    chassisPower = *((float *)&power_uint32);
    chassisPowerLimit   = (data[6] << 8) | data[5];
    capEnergy = data[7];
	
	SuperCap_Data->StatusCode = statusCode;
    SuperCap_Data->ChassisPower = chassisPower;
    SuperCap_Data->ChassisPowerLimit   = chassisPowerLimit;
    SuperCap_Data->CapEnergy = capEnergy;
}

Chassis_Power_instance_t *Chassis_Power_Init(void)
{
    Chassis_Power_instance_t *instance = (Chassis_Power_instance_t *)malloc(sizeof(Chassis_Power_instance_t));
    if (instance == NULL)
    {
        return NULL;
    }
    memset(instance, 0, sizeof(Chassis_Power_instance_t));

    instance->ChassisPower = 0.0f;
    instance->ChassisPowerLimit   = 0.0f;
    instance->CapEnergy = 0;
    // 初始化CAN实例
    can_init_config_t Chassis_power_can_config;
	Chassis_power_can_config.can_handle = &hfdcan2;
    Chassis_power_can_config.tx_id = 0x06A;
    Chassis_power_can_config.rx_id = 0x05A;
    Chassis_power_can_config.can_module_callback = Chassis_Power_Decode;
    Chassis_power_can_config.id = (void *)instance;

    instance->Chassis_power_can_instance = CAN_Register(&Chassis_power_can_config);
    if (instance->Chassis_power_can_instance == NULL)
    {
        free(instance);
        return NULL;
    }

    return instance;
}

void SuperCap_send_cmd(uint16_t power_limit, uint16_t RefereeEnergyBuffer)
{
    uint8_t tx_buff[8] = {0};

    // tx_buff[0]：使能DCDC、系统重启位和清除错误位默认0，保留位0
    tx_buff[0] = 0x00;

    // tx_buff[1-2]：裁判系统功率限制（16位）
    tx_buff[1] = POWER_LIMIT & 0xFF;
    tx_buff[2] = (POWER_LIMIT >> 8) & 0xFF;

    // tx_buff[3-4]：裁判系统缓冲能量（16位）
    tx_buff[3] = ENERGY_BUFFER & 0xFF;
    tx_buff[4] = (ENERGY_BUFFER >> 8) & 0xFF;

    // tx_buff[5-7]：保留位/状态位，默认0
    tx_buff[5] = 0x00;
    tx_buff[6] = 0x00;
    tx_buff[7] = 0x00;

    memcpy(SuperCap_Data->Chassis_power_can_instance->tx_buff, tx_buff, sizeof(tx_buff));
    CAN_Transmit(SuperCap_Data->Chassis_power_can_instance, 1);
}

void SuperCap_Enable(uint16_t power_limit, uint16_t RefereeEnergyBuffer)
{
	uint8_t tx_buff[8] = {0};

    // tx_buff[0]：使能DCDC位置1，其余位0
    tx_buff[0] = 0x01;

    // tx_buff[1-2]：裁判系统功率限制（16位）
    tx_buff[1] = POWER_LIMIT & 0xFF;
    tx_buff[2] = (POWER_LIMIT >> 8) & 0xFF;

    // tx_buff[3-4]：裁判系统缓冲能量（16位）
    tx_buff[3] = ENERGY_BUFFER & 0xFF;
    tx_buff[4] = (ENERGY_BUFFER >> 8) & 0xFF;

    // tx_buff[5-7]：保留位/状态位，默认0
    tx_buff[5] = 0x00;
    tx_buff[6] = 0x00;
    tx_buff[7] = 0x00;

    memcpy(SuperCap_Data->Chassis_power_can_instance->tx_buff, tx_buff, sizeof(tx_buff));
    CAN_Transmit(SuperCap_Data->Chassis_power_can_instance, 1);
}

void SuperCap_Disable(uint16_t power_limit, uint16_t RefereeEnergyBuffer)
{
	uint8_t tx_buff[8] = {0}; 

    // tx_buff[0]：使能DCDC位置0，其余位0
    tx_buff[0] = 0x00;

    // tx_buff[1-2]：裁判系统功率限制（16位）
    tx_buff[1] = POWER_LIMIT & 0xFF;
    tx_buff[2] = (POWER_LIMIT >> 8) & 0xFF;

    // tx_buff[3-4]：裁判系统缓冲能量（16位）
    tx_buff[3] = ENERGY_BUFFER & 0xFF;
    tx_buff[4] = (ENERGY_BUFFER >> 8) & 0xFF;

    // tx_buff[5-7]：保留位/状态位，默认0
    tx_buff[5] = 0x00;
    tx_buff[6] = 0x00;
    tx_buff[7] = 0x00;

    memcpy(SuperCap_Data->Chassis_power_can_instance->tx_buff, tx_buff, sizeof(tx_buff));
    CAN_Transmit(SuperCap_Data->Chassis_power_can_instance, 1);
}

void SuperCap_SystemRestart(uint16_t power_limit, uint16_t RefereeEnergyBuffer)
{
	uint8_t tx_buff[8] = {0};

    // tx_buff[0]：系统重启位置1，其余位0
    tx_buff[0] = 0x02;

    // tx_buff[1-2]：裁判系统功率限制（16位）
    tx_buff[1] = POWER_LIMIT & 0xFF;
    tx_buff[2] = (POWER_LIMIT >> 8) & 0xFF;

    // tx_buff[3-4]：裁判系统缓冲能量（16位）
    tx_buff[3] = ENERGY_BUFFER & 0xFF;
    tx_buff[4] = (ENERGY_BUFFER >> 8) & 0xFF;

    // tx_buff[5-7]：保留位/状态位，默认0
    tx_buff[5] = 0x00;
    tx_buff[6] = 0x00;
    tx_buff[7] = 0x00;

    memcpy(SuperCap_Data->Chassis_power_can_instance->tx_buff, tx_buff, sizeof(tx_buff));
    CAN_Transmit(SuperCap_Data->Chassis_power_can_instance, 1);
}

void SuperCap_ClearError(uint16_t power_limit, uint16_t RefereeEnergyBuffer)
{
	uint8_t tx_buff[8] = {0};

    // tx_buff[0]：清除错误位置1，其余位0
    tx_buff[0] = 0x04;

    // tx_buff[1-2]：裁判系统功率限制（16位）
    tx_buff[1] = POWER_LIMIT & 0xFF;
    tx_buff[2] = (POWER_LIMIT >> 8) & 0xFF;

    // tx_buff[3-4]：裁判系统缓冲能量（16位）
    tx_buff[3] = ENERGY_BUFFER & 0xFF;
    tx_buff[4] = (ENERGY_BUFFER >> 8) & 0xFF;

    // tx_buff[5-7]：保留位/状态位，默认0
    tx_buff[5] = 0x00;
    tx_buff[6] = 0x00;
    tx_buff[7] = 0x00;

    memcpy(SuperCap_Data->Chassis_power_can_instance->tx_buff, tx_buff, sizeof(tx_buff));
    CAN_Transmit(SuperCap_Data->Chassis_power_can_instance, 1);
}