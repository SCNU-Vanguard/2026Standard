/*
 * @file		Serial.c/h
 * @brief		usb-cdc通信程序，用于上位机和stm32的通信，需配合上位机代码使用
 * @history
 * 版本			作者			编写日期
 * v1.1.0		许金帅		2023/4/11
 * v3.0.0   miracle  2025/12/9
 * v3.3.0   miracle  2026/1/21   (修正导航包和视觉包的crc校验问题,并增加新视觉包(TJ)的支持)
 */
#include "Serial.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "INS.h"
#include "CRC.h"
#include "stdint.h"

nv_receive_packet_t nv_aim_packet_from_nuc;
nv_send_packet_t nv_aim_packet_to_nuc;

vs_receive_packet_t vs_aim_packet_from_nuc;
vs_send_packet_t vs_aim_packet_to_nuc;

receive_packet_t aim_packet_from_nuc;
send_packet_t aim_packet_to_nuc;

// uint8_t nv_buf_receive_from_nuc[sizeof(nv_receive_packet_t)];
// uint8_t vs_buf_receive_from_nuc[sizeof(vs_receive_packet_t)];
// uint8_t buf_receive_from_nuc[sizeof(receive_packet_t)];

/**
* @brief  将数据包发送到上位机
  * @param
            Fed 数据包
            Len 数据包的大小--占用内存字节数（协议规定为8Bytes）
  * @retval 无
  */
int CDC_SendFeed(uint8_t *Fed, uint16_t Len)
{
    CDC_Transmit_HS(Fed, Len);
    return 0;
}

/**
  * @brief  将接收到的控制数组拆分为对应的结构体(导航)
  * @param
        receive_buf 接收到的原始数据数组
        receive_packet 接收数据的的储存结构体
        Len 接收数组的长度
  * @retval 无
  */
void NV_UnPack_Data_ROS2(uint8_t *receive_buf, nv_receive_packet_t *receive_packet, uint16_t Len)
{
    if (receive_buf[0] == 0xA5)
    {
        uint16_t w_expected;
        w_expected = Get_CRC16_Check_Sum(receive_buf, Len - 2, 0xFFFF);
        if ((w_expected & 0xff) == receive_buf[Len - 2] && ((w_expected >> 8) & 0xff) == receive_buf[Len - 1])
        {
            memcpy(receive_packet, receive_buf, Len);
        }
    }
    memset(receive_buf, 0, Len);
}

/**
  * @brief  将接收到的控制数组拆分为对应的结构体(视觉)
  * @param
        receive_buf 接收到的原始数据数组
        receive_packet 接收数据的的储存结构体
        Len 接收数组的长度
  * @retval 无
  */
void VS_UnPack_Data_ROS2(uint8_t *receive_buf, vs_receive_packet_t *receive_packet, uint16_t Len)
{
    // /*Len为原始接收数据长度，如果要排除末尾的换行符，则需要减去最后一位数据包*/
    // uint16_t actual_Len = Len - 1
    uint16_t w_expected;
    w_expected = Get_CRC16_Check_Sum(receive_buf, Len - 2, 0xFFFF);
    if ((w_expected & 0xff) == receive_buf[Len - 2] && ((w_expected >> 8) & 0xff) == receive_buf[Len - 1]) // CRC检验出了问题
    {
        memcpy(receive_packet, receive_buf, Len);
    }
    // memset(receive_buf, 0, Len);
}

/**
  * @brief  将接收到的控制数组拆分为对应的结构体(旧版)
  * @param
        receive_buf 接收到的原始数据数组
        receive_packet 接收数据的的储存结构体
        Len 接收数组的长度
  * @retval 无
  */
void UnPack_Data_ROS2(uint8_t *receive_buf, receive_packet_t *receive_packet, uint16_t Len)
{
    if (receive_buf[0] == 0xA5)
    {
        uint16_t w_expected;
        w_expected = Get_CRC16_Check_Sum(receive_buf, Len - 2, 0xFFFF);
        if ((w_expected & 0xff) == receive_buf[Len - 2] && ((w_expected >> 8) & 0xff) == receive_buf[Len - 1])
        {
            memcpy(receive_packet, receive_buf, Len);
        }
    }
    memset(receive_buf, 0, Len);
}

/**
  * @brief  将准备发送的数据结构体拆为数组并发送(导航)
  * @param
        send_packet 发送数据的的储存结构体
  * @retval 无
  */
void NV_Pack_And_Send_Data_ROS2(nv_send_packet_t *send_packet)
{

    uint16_t len = sizeof(nv_send_packet_t);
    uint8_t tmp[len];
    memcpy(tmp, send_packet, len - 2);
    uint16_t w_crc = Get_CRC16_Check_Sum(tmp, len - 2, 0xFFFF);

    tmp[len - 2] = (uint8_t)(w_crc & 0x00ff);
    tmp[len - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
    CDC_SendFeed(tmp, len);
}

/**
  * @brief  将准备发送的数据结构体拆为数组并发送(视觉)
  * @param
        send_packet 发送数据的的储存结构体
  * @retval 无
  */
void VS_Pack_And_Send_Data_ROS2(vs_send_packet_t *send_packet)
{

    uint16_t len = sizeof(vs_send_packet_t);
    uint8_t tmp[len];
    memcpy(tmp, send_packet, len - 2);
    uint16_t w_crc = Get_CRC16_Check_Sum(tmp, len - 2, 0xFFFF);

    tmp[len - 2] = (uint8_t)(w_crc & 0x00ff);
    tmp[len - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);

    CDC_SendFeed(tmp, len);
    // /*以下内容是为了配合nx视觉的SZU的代码在包尾增加一个换行符，不影响CRC校验的值*/
    // // 创建新缓冲区：原始数据 + CRC + 换行符
    // uint8_t tmp1[len + 1]; // 增加2个字节用于换行符
    // // 复制原始数据+CRC到新缓冲区
    // memcpy(tmp1, tmp, len);
    // tmp1[len] = 0x0A; // 换行符 '\n'

    // // 发送带换行符的数据包
    // CDC_SendFeed(tmp1, len + 1);
}

/**
  * @brief  将准备发送的数据结构体拆为数组并发送(旧版)
  * @param
        send_packet 发送数据的的储存结构体
  * @retval 无
  */
void Pack_And_Send_Data_ROS2(send_packet_t *send_packet)
{
    uint16_t len = sizeof(send_packet_t);
    uint8_t tmp[sizeof(send_packet_t)];
    memcpy(tmp, send_packet, len - 2);
    uint16_t w_crc = Get_CRC16_Check_Sum(tmp, len - 2, 0xFFFF);

    tmp[len - 2] = (uint8_t)(w_crc & 0x00ff);
    tmp[len - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
    CDC_SendFeed(tmp, len);
}

void NV_Send_Packet_Init(nv_send_packet_t *send_packet)
{
    send_packet->header = 0x5A; // 帧头赋值
    send_packet->imu_pitch = INS.Pitch;
    send_packet->imu_yaw = INS.Yaw;
}

/* 初始化发送给上位机的视觉数据包 */
void VS_Send_Packet_Init(vs_send_packet_t *send_packet)
{
    /*TJ*/
    send_packet->head[0] = 'S';
    send_packet->head[1] = 'P';
    send_packet->mode = 0;
    send_packet->q[0] = 0.0f;
    send_packet->q[1] = 0.0f;
    send_packet->q[2] = 0.0f;
    send_packet->q[3] = 0.0f;
    send_packet->yaw = 0.0f;
    send_packet->yaw_vel = 0.0f;
    send_packet->pitch = 0.0f;
    send_packet->pitch_vel = 0.0f;
    send_packet->bullet_speed = 0;
    send_packet->bullet_count = 0;
    send_packet->is_play = 0;
    send_packet->game_time = 0.0f;
    send_packet->own_hp = 0.0f;

}

/* 初始化接收自上位机的视觉数据包 */
void VS_Receive_Packet_Init(vs_receive_packet_t *receive_packet)
{

    receive_packet->head[0] = 'S';
    receive_packet->head[1] = 'P';
    receive_packet->mode = 0;
    receive_packet->yaw = 0.0f;
    receive_packet->yaw_vel = 0.0f;
    receive_packet->yaw_acc = 0.0f;
    receive_packet->pitch = 0.0f;
    receive_packet->pitch_vel = 0.0f;
    receive_packet->pitch_acc = 0.0f;
    receive_packet->vx = 0.0f;
    receive_packet->vy = 0.0f;
    receive_packet->posture = 0;
    receive_packet->circle = 0;
    receive_packet->crc16 = 0;
}

/* 初始化发送给上位机的总数据包 */
void Send_Packet_Init(send_packet_t *send_packet)
{
    send_packet->header = 0x5A;
    send_packet->detect_color = 0;
    send_packet->task_mode = 0;
    send_packet->reset_tracker = 0;
    send_packet->is_play = 0;
    send_packet->change_target = 0;
    send_packet->reserved = 0;
    send_packet->roll = 0.0f;
    send_packet->pitch = 0.0f;
    send_packet->yaw = 0.0f;
    send_packet->aim_x = 0.0f;
    send_packet->aim_y = 0.0f;
    send_packet->aim_z = 0.0f;
    send_packet->game_time = 0;
    send_packet->timestamp = 0;
    send_packet->checksum = 0;
}