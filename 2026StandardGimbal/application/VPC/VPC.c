/*
 * VPC.c
 * author: miracle-cloud
 * Created on: 2025年10月31日
 */

#include "VPC.h"
#include "Serial.h"
#include "INS.h"
#include "gimbal.h"

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"
#include "usbd_cdc_if.h"
#include "rs485.h"
#include "shoot.h"
uint8_t frame_buf[512];
Quaternion_t motor_q;
void VPC_Init(void)
{
  VS_Send_Packet_Init(&vs_aim_packet_to_nuc);
  VS_Receive_Packet_Init(&vs_aim_packet_from_nuc);
}

/* 更新发送给上位机的数据包 */
void VPC_UpdatePackets(void)
{
  /*哨兵传输数据区*/
  vs_aim_packet_to_nuc.head[0] = 'S';
  vs_aim_packet_to_nuc.head[1] = 'P';
  vs_aim_packet_to_nuc.mode = 1;

  vs_aim_packet_to_nuc.q[0] = INS.q[0];
  vs_aim_packet_to_nuc.q[1] = INS.q[1];
  vs_aim_packet_to_nuc.q[2] = INS.q[2];
  vs_aim_packet_to_nuc.q[3] = INS.q[3];
  vs_aim_packet_to_nuc.yaw = INS.Yaw;
  vs_aim_packet_to_nuc.yaw_vel = uart2_rx_message.yaw_vel; // 来自底盘的yaw角速度
  vs_aim_packet_to_nuc.pitch = INS.Pitch;
  vs_aim_packet_to_nuc.pitch_vel = gimbal_motor_pitch->measure.speed; // 云台pitch轴角速度 rad/s

  vs_aim_packet_to_nuc.is_play = uart2_rx_message.is_play;     // uart2_rx_message.is_play;
  vs_aim_packet_to_nuc.game_time = uart2_rx_message.game_time; //  uart2_rx_message.game_time; // 当前阶段剩余时间

  vs_aim_packet_to_nuc.positon_x = uart2_rx_message.positon_x; // 机器人自身位置
  vs_aim_packet_to_nuc.positon_y = uart2_rx_message.positon_y;
  vs_aim_packet_to_nuc.tar_positon_x = uart2_rx_message.tar_positon_x; // 目标地点位置（半自动模式使用）
  vs_aim_packet_to_nuc.tar_positon_y = uart2_rx_message.tar_positon_y;

  vs_aim_packet_to_nuc.own_hp = uart2_rx_message.own_hp;                 // 己方血量
  vs_aim_packet_to_nuc.outpost_HP = uart2_rx_message.outpost_HP;         // 己方前哨站血量
  vs_aim_packet_to_nuc.outpost_status = uart2_rx_message.outpost_status; // 前哨站占领状态
  vs_aim_packet_to_nuc.fort_status = uart2_rx_message.fort_status;       // 堡垒占领状态
  vs_aim_packet_to_nuc.bullet_speed = BULLET_V;                          // 现在为设置弹速，不是实际弹速
  vs_aim_packet_to_nuc.bullet_count = 0;                                 // 未定
}

/*根据帧头选择对应的数据处理*/
void Choose_VPC_Type(void)
{
  uint16_t frame_len = cdc_rx_len;

  /* 拷贝完整帧 */
  memcpy(frame_buf, cdc_rx_cache, frame_len);

  /*视觉部分（同济大学版本）*/
  if (frame_buf[0] == 'S' && frame_buf[1] == 'P')
  {
    VS_UnPack_Data_ROS2(frame_buf, &vs_aim_packet_from_nuc, sizeof(vs_receive_packet_t));
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_xSemVPC, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}