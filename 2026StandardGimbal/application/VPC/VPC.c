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
int hp = 1;
void VPC_Init(void)
{
  NV_Send_Packet_Init(&nv_aim_packet_to_nuc);
  VS_Send_Packet_Init(&vs_aim_packet_to_nuc);
  VS_Receive_Packet_Init(&vs_aim_packet_from_nuc);
  // Send_Packet_Init(&aim_packet_to_nuc);
}

/*
根据云台的电机解算imu角度逆解算出四元数
*/
void MotorToQuaternion(Quaternion_t *q, float motor_yaw, float motor_pitch)
{
   // 1. 获取半角
    float half_y = motor_yaw * 0.5f;   
    float half_p = motor_pitch * 0.5f; 

    float cy = cosf(half_y);
    float sy = sinf(half_y);
    float cp = cosf(half_p);
    float sp = sinf(half_p);

    // 2. 修正后的分量分配
    // w, x 保持原样 (与 IMU 一致)
    q->w = cp * cy;
    q->x = sp * cy;
    
    // y, z 翻转正负号 (适配你的 IMU 解算结果)
    // 原来是负号，现在改为正号即可实现反向
    q->y = sp * sy;  
    q->z = cp * sy;  

    // 3. 归一化 (必须保留，防止浮点误差导致四元数失效)
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 0.000001f) {
        float invNorm = 1.0f / norm;
        q->w *= invNorm;
        q->x *= invNorm;
        q->y *= invNorm;
        q->z *= invNorm;
    }
}


/* 更新发送给上位机的数据包 */
void VPC_UpdatePackets(void)
{
	if(hp <500)
	{
		hp++;
	}
	else if(hp>=500)
		hp = 0;
  /*导航传输数据区*/
  nv_aim_packet_to_nuc.header = 0x5A; // 帧头赋值
// nv_aim_packet_to_nuc.detect_color = 1;
//	 nv_aim_packet_to_nuc.task_mode = 0;
//	 nv_aim_packet_to_nuc.reset_tracker = 1;
//	 nv_aim_packet_to_nuc.is_play = 0;
//	 nv_aim_packet_to_nuc.change_target = 1;
//	 nv_aim_packet_to_nuc.reserve = 2;
  nv_aim_packet_to_nuc.imu_pitch = INS.Pitch;
  nv_aim_packet_to_nuc.imu_yaw = INS.Yaw;

  nv_aim_packet_to_nuc.joint_pitch = -gimbal_motor_pitch->measure.rad + 5.22f;
  nv_aim_packet_to_nuc.joint_yaw = angle_yaw_motor2imu;
  
	nv_aim_packet_to_nuc.aim_x = 0;
  nv_aim_packet_to_nuc.aim_y = 0;
  nv_aim_packet_to_nuc.aim_z = 0;
  nv_aim_packet_to_nuc.timestamp = 0; // 时间戳 （未定）
  nv_aim_packet_to_nuc.robot_hp = hp;  // 血量（未定）
  nv_aim_packet_to_nuc.game_time = 0; // 比赛时间（未定）


  /*新视觉传输数据区*/                //(同济大学版本)

  vs_aim_packet_to_nuc.head[0] = 'S';
  vs_aim_packet_to_nuc.head[1] = 'P';
  vs_aim_packet_to_nuc.mode = 1;
  // vs_aim_packet_to_nuc.q[0] = motor_q.w;
  // vs_aim_packet_to_nuc.q[1] = motor_q.x;
  // vs_aim_packet_to_nuc.q[2] = motor_q.y;
  // vs_aim_packet_to_nuc.q[3] = motor_q.z;
  // vs_aim_packet_to_nuc.yaw = angle_yaw_motor2imu;
  // vs_aim_packet_to_nuc.yaw_vel = uart2_rx_message.yaw_vel; // 来自底盘的yaw角速度
  // vs_aim_packet_to_nuc.pitch = angle_pitch_motor2imu;
  // vs_aim_packet_to_nuc.pitch_vel = gimbal_motor_pitch->measure.speed; // 云台pitch轴角速度 rad/s

  vs_aim_packet_to_nuc.q[0] = INS.q[0];
  vs_aim_packet_to_nuc.q[1] = INS.q[1];
  vs_aim_packet_to_nuc.q[2] = INS.q[2];
  vs_aim_packet_to_nuc.q[3] = INS.q[3];
  vs_aim_packet_to_nuc.yaw = INS.Yaw;
  vs_aim_packet_to_nuc.yaw_vel = uart2_rx_message.yaw_vel; // 来自底盘的yaw角速度
  vs_aim_packet_to_nuc.pitch = INS.Pitch;
  vs_aim_packet_to_nuc.pitch_vel = gimbal_motor_pitch->measure.speed; // 云台pitch轴角速度 rad/s

  vs_aim_packet_to_nuc.bullet_speed = BULLET_V;                       // 现在为设置弹速，不是实际弹速
  vs_aim_packet_to_nuc.bullet_count = 0;                              // 未定

}

/*根据帧头选择对应的数据处理*/
void Choose_VPC_Type(void)
{
  uint16_t frame_len = cdc_rx_len;

  /* 拷贝完整帧 */
  memcpy(frame_buf, cdc_rx_cache, frame_len);

  /*根据帧头来判断接收到的是哪个数据包*/
  /*导航部分*/
  if (frame_buf[0] == 0xA5)
  {
     NV_UnPack_Data_ROS2(frame_buf, &nv_aim_packet_from_nuc, sizeof(nv_receive_packet_t));
    //UnPack_Data_ROS2(frame_buf, &aim_packet_from_nuc, sizeof(receive_packet_t));
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_xSemVPC, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  /*视觉部分（同济大学版本）*/
  else if (frame_buf[0] == 'S' && frame_buf[1] == 'P')
  {
    VS_UnPack_Data_ROS2(frame_buf, &vs_aim_packet_from_nuc, sizeof(vs_receive_packet_t));
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_xSemVPC, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}