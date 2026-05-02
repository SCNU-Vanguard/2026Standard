#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "referee_protocol.h"

#include "defense_center.h"

#include "bsp_usart.h"

#if REFEREE_RAW
#else
extern uint8_t UI_Seq;

#pragma pack(1)
typedef struct
{
  uint8_t Robot_Color;		// 机器人颜色
  uint16_t Robot_ID;			// 本机器人ID
  uint16_t Cilent_ID;			// 本机器人对应的客户端ID
  uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
  referee_id_t referee_id;

  xFrameHeader FrameHeader;
  uint16_t CmdID;
  ext_game_state_t GameState;                      // 0x0001
  ext_game_result_t GameResult;                    // 0x0002
  ext_game_robot_HP_t GameRobotHP;                 // 0x0003
  ext_event_data_t EventData;                      // 0x0101
  // ext_supply_projectile_action_t SupplyProjectileAction; // 0x0102
  ext_referee_warning_t RefereeWarning;            // 0x0104
  ext_dart_info_t DartInfo;                        // 0x0105
  ext_robot_performance_t RobotPerformance;        // 0x0201
  ext_power_heat_data_t PowerHeatData;             // 0x0202
  ext_game_robot_pos_t GameRobotPos;               // 0x0203
  ext_buff_musk_t BuffMusk;                        // 0x0204
  // aerial_robot_energy_t AerialRobotEnergy;			   // 0x0205
  ext_robot_hurt_t RobotHurt;                      // 0x0206
  ext_shoot_data_t ShootData;                      // 0x0207
  ext_projectile_allowance_t ProjectileAllowance;  // 0x0208
  ext_rfid_status_t RFIDStatus;                    // 0x0209
  ext_dart_launch_status_t DartLaunchStatus;       // 0x020A
  ext_ground_robot_position_t GroundRobotPosition; // 0x020B
  ext_radar_mark_data_t RadarMarkData;             // 0x020C
  ext_sentry_info_t SentryInfo;                    // 0x020D
  ext_radar_info_t RadarInfo;                      // 0x020E
  ext_map_command_t MapCommand;                      // 0x0303
  // 自定义交互数据的接收
  Communicate_ReceiveData_t ReceiveData;

  uint8_t init_flag;

} referee_info_t;

// 模式是否切换标志位，0为未切换，1为切换，static定义默认为0
typedef struct
{
  uint32_t control_flag :1;
  uint32_t chassis_flag :1;
  uint32_t gimbal_flag :1;
  uint32_t shoot_flag :1;
  uint32_t ammo_flag :1;
  uint32_t friction_flag :1;
  uint32_t Power_flag :1;
  uint32_t Super_flag :1;
} Referee_Interactive_Flag_t;

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
typedef struct
{
  Referee_Interactive_Flag_t Referee_Interactive_Flag;
  // 为UI绘制以及交互数据所用
  uint8_t control_mode;
  uint8_t chassis_mode;			 // 底盘模式
  uint8_t gimbal_mode;				 // 云台模式
  uint8_t shoot_mode;				 // 发射模式设置
  uint8_t friction_mode;			 // 摩擦轮关闭
  uint8_t ammo_mode;					 // 弹舱盖打开
  float Chassis_Power_Limit; // 功率控制
  uint8_t Super_Power;

  // 上一次的模式，用于flag判断
  uint8_t control_last_mode;
  uint8_t chassis_last_mode;
  uint8_t gimbal_last_mode;
  uint8_t shoot_last_mode;
  uint8_t friction_last_mode;
  uint8_t ammo_last_mode;
  uint8_t Chassis_last_Power_Limit;
  uint8_t Super_last_Power;

} Referee_Interactive_info_t;

#pragma pack()

/**
 * @brief 裁判系统通信初始化,该函数会初始化裁判系统串口,开启中断
 *
 * @param referee_usart_handle 串口handle,C板一般用串口6
 * @return referee_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
 */
referee_info_t* Referee_Register(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
 * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */
void Referee_Send(uint8_t *send, uint16_t tx_len);
#endif
#endif // !REFEREE_H
