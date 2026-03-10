/*
 * @file		Serial.c/h
 * @brief		usb-CDC通信程序，用于上位机和stm32的通信，需配合上位机代码使用，现采用zy版本
 * @history
 * 版本			作者			编写日期
 * v1.0.0		许金帅		2023/4/1
 * v2.0.0		冯俊玮		2024/10/29
 * v3.0.0   miracle    2025/12/6  （将传统的一个发送包拆为两个包发送，一个给视觉，一个给导航）
 * v3.3.0   miracle    2026/1/21   (修正导航包和视觉包的crc校验问题,并增加新视觉包(TJ)的支持)
 */
#ifndef __Serial_h__
#define __Serial_h__
#include "stm32h7xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"
// 下面是为和ROS2上位机沟通而定制的结构体(zy)

typedef struct
{
  uint8_t sof;
  uint8_t crc8;
} __attribute__((packed)) FrameHeader_t;

typedef struct
{
  uint16_t crc16;
} __attribute__((packed)) FrameTailer_t;

typedef struct
{
  uint64_t config;      // 配置参数，通常用于传递模式、开关、标志位等综合配置信息（如bit位控制不同功能）
  float target_pose[3]; // 目标三维坐标，通常为目标在世界/相机坐标系下的 x, y, z
  float curr_yaw;       // 当前云台的 yaw 角度（水平旋转角）
  float curr_pitch;     // 当前云台的 pitch 角度（俯仰角）
  uint8_t enemy_color;  // 敌方颜色（如 0=蓝，1=红），用于自瞄识别目标阵营
  uint8_t shoot_config; // 射击配置，通常用于控制射击模式、发弹策略等
} __attribute__((packed)) OutputData_t;

typedef struct
{
  float shoot_yaw;   // 经过自瞄预测的yaw角度
  float shoot_pitch; // 经过自瞄预测的pitch角度
  uint8_t fire;      // 是否发弹
} __attribute__((packed)) InputData_t;

// /*导航的发送结构体aim to nuc*/
// typedef struct
// {
//   uint8_t header;           // 0x5A
//   uint8_t detect_color : 1; // 0-red 1-blue
//   uint8_t task_mode : 2;    // 0-auto 1-aim 2-buff
//   bool reset_tracker : 1;
//   uint8_t is_play : 1;
//   uint8_t change_target : 1;
//   uint8_t reserve : 2;

//   float imu_yaw;
//   float imu_pitch;

//   float joint_yaw;
//   float joint_pitch;
//   float aim_x;
//   float aim_y;
//   float aim_z;
//   uint16_t timestamp; // (ms) board time
//   uint16_t robot_hp;
//   uint16_t game_time; // (s) game time [0, 450]
//   uint16_t checksum;

// } __attribute__((packed)) nv_send_packet_t;

// /*导航的接收结构体aim from nuc*/
// typedef struct
// {
//   uint8_t header; // 0xA5   帧头

//   uint8_t state : 2;      // 0-untracking 1-tracking-aim 2-tracking-buff
//   uint8_t id : 3;         // aim: 0-outpost 6-guard 7-base
//   uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
//   // uint8_t reserved : 1;
//   float yaw;
//   float pitch;
//   float yaw_diff;
//   float pitch_diff;

//   int fire_advice;

//   float vx; // x轴速度指令
//   float vy; // y轴速度指令

//   // float joint_yaw;
//   // float joint_pitch;
//   uint16_t checksum;
// } __attribute__((packed)) nv_receive_packet_t;


/*导航的发送结构体aim to nuc*/   //(导航与TJ视觉合并包)
typedef struct
{
  uint8_t header;           // 0x5A
 uint8_t detect_color : 1; // 0-red 1-blue
 uint8_t task_mode : 2;    // 0-auto 1-aim 2-buff
 bool reset_tracker : 1;
 uint8_t is_play : 1;  
 uint8_t change_target : 1;  //切换目标
 uint8_t reserve : 2;  //预留位

  float imu_yaw;
  float imu_pitch;

  float joint_yaw;
  float joint_pitch;
  
  uint16_t aim_x;
  uint16_t aim_y;
  uint16_t aim_z;
  
  uint16_t timestamp; // (ms) board time
  uint16_t robot_hp;
  uint16_t game_time; // (s) game time [0, 450]
  uint16_t checksum;

} __attribute__((packed)) nv_send_packet_t;

/*导航的接收结构体aim from nuc*/   //(导航与TJ视觉合并包)
typedef struct
{
  uint8_t header; // 0xA5   帧头

  uint8_t state : 2;      // 0-untracking 1-tracking-aim 2-tracking-buff
  uint8_t id : 3;         // aim: 0-outpost 6-guard 7-base
  uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
  // uint8_t reserved : 1;
  float yaw;
  float pitch;
  float yaw_diff;
  float pitch_diff;

  int fire_advice;

  float vx; // x轴速度指令
  float vy; // y轴速度指令
  //uint8_t posture; // 0-移动姿态 1-进攻姿态 2-防御姿态

  uint16_t checksum;
} __attribute__((packed)) nv_receive_packet_t;




// /*传统视觉的发送结构体aim to nuc*/
// typedef struct
// {
//   uint8_t header;           // 0x5A
//   uint8_t detect_color : 1; // 0-red 1-blue
//   uint8_t task_mode : 2;    // 0-auto 1-aim 2-buff
//   uint8_t reset_tracker : 1;
//   uint8_t is_play : 1;
//   uint8_t change_target : 1;
//   uint8_t reserved : 2;
//   float roll;
//   float pitch;
//   float yaw;
//   float aim_x;
//   float aim_y;
//   float aim_z;
//   uint16_t game_time; // (s) game time [0, 450]
//   uint32_t timestamp; // (ms) board time
//   uint16_t checksum;
// } __attribute__((packed)) vs_send_packet_t;

// /*传统视觉的接收结构体aim from nuc*/
// typedef struct
// {
//   uint8_t header;         // 0xA5
//   uint8_t state : 2;      // 0-untracking 1-tracking-aim 2-tracking-buff
//   uint8_t id : 3;         // aim: 0-outpost 6-guard 7-base
//   uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal

//   // o-auto
//   float yaw;
//   float pitch;

//   float yaw_diff;
//   float pitch_diff;
//   int fire_advice;

//   uint16_t checksum;
// } __attribute__((packed)) vs_receive_packet_t;

/*新视觉的发送结构体aim to nuc*/
typedef struct
{
  /*SZ*/
  // FrameHeader_t frame_header;
  // OutputData_t output_data;
  // FrameTailer_t frame_tailer;

  /*TJ*/
  uint8_t head[2]; // = {'S', 'P'};
  uint8_t mode;    // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];      // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count; // 子弹累计发送次数
  uint16_t crc16;

} __attribute__((packed)) vs_send_packet_t;

/*新视觉的接收结构体aim from nuc*/
typedef struct
{
  // FrameHeader_t frame_header;
  // InputData_t input_data;
  // FrameTailer_t frame_tailer;

  uint8_t head[2]; // = {'S', 'P'};
  uint8_t mode;    // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
} __attribute__((packed)) vs_receive_packet_t;

/*单包发送结构体*/
typedef struct
{
  uint8_t header;           // 0x5A
  uint8_t detect_color : 1; // 0-red 1-blue
  uint8_t task_mode : 2;    // 0-auto 1-aim 2-buff
  uint8_t reset_tracker : 1;
  uint8_t is_play : 1;
  uint8_t change_target : 1;
  uint8_t reserved : 2;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  
  uint16_t game_time; // (s) game time [0, 450]
  uint32_t timestamp; // (ms) board time
  uint16_t checksum;

} __attribute__((packed)) send_packet_t;

/*单包接收结构体*/
typedef struct
{
  uint8_t header;         // 0xA5
  uint8_t state : 2;      // 0-untracking 1-tracking-aim 2-tracking-buff
  uint8_t id : 3;         // aim: 0-outpost 6-guard 7-base
  uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal

  // o-auto
  float yaw;
  float pitch;

  float yaw_diff;

  float pitch_diff;
  int fire_advice;

  uint16_t checksum;
} __attribute__((packed)) receive_packet_t;

extern nv_receive_packet_t nv_aim_packet_from_nuc;
extern nv_send_packet_t nv_aim_packet_to_nuc;

extern vs_receive_packet_t vs_aim_packet_from_nuc;
extern vs_send_packet_t vs_aim_packet_to_nuc;

extern receive_packet_t aim_packet_from_nuc;
extern send_packet_t aim_packet_to_nuc;

extern FrameHeader_t frame_header;
extern InputData_t input_data;
extern FrameTailer_t frame_tailer;

extern uint8_t nv_buf_receive_from_nuc[sizeof(nv_receive_packet_t)]; // 具体应用在usb接收函数中（在CDC_if.c中）
extern uint8_t vs_buf_receive_from_nuc[sizeof(vs_receive_packet_t)]; // 具体应用在usb接收函数中（在CDC_if.c中）
extern uint8_t buf_receive_from_nuc[sizeof(receive_packet_t)];       // 具体应用在usb接收函数中（在CDC_if.c中）

int CDC_SendFeed(uint8_t *Fed, uint16_t Len); // CDC发送反馈数据的函数
// ROS2发送代码
void NV_Pack_And_Send_Data_ROS2(nv_send_packet_t *send_packet);
void NV_UnPack_Data_ROS2(uint8_t *receive_buf, nv_receive_packet_t *receive_packet, uint16_t Len);
void NV_Send_Packet_Init(nv_send_packet_t *send_packet);

void VS_Pack_And_Send_Data_ROS2(vs_send_packet_t *send_packet);
void VS_UnPack_Data_ROS2(uint8_t *receive_buf, vs_receive_packet_t *receive_packet, uint16_t Len);
void VS_Send_Packet_Init(vs_send_packet_t *send_packet);
void VS_Receive_Packet_Init(vs_receive_packet_t *receive_packet);
void Pack_And_Send_Data_ROS2(send_packet_t *send_packet);
void UnPack_Data_ROS2(uint8_t *receive_buf, receive_packet_t *receive_packet, uint16_t Len);
void Send_Packet_Init(send_packet_t *send_packet);

// void Analysis_VPC_Buffer(void);

// int CDC_Receive_ROS2(uint8_t *receive_buf, receive_packet_t *receive_packet, uint32_t Len);

#endif
