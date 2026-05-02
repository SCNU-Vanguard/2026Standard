
#include <string.h>
#include "main.h"
#include "sentry_info.h"
#include "cmsis_os2.h"
#include "referee_task.h"
#include "rs485.h"
#include "CRC.h"

uint8_t seq = 0;
uint16_t ui_self_id = 107; // 红方哨兵7，蓝方哨兵107
// 为了使消息能够发送给裁判系统，必须要自定义ui_self_id变量（该变量也可以使用其他代码从裁判系统读取后由程序修改）

sentry_msg_t sentry_msg;             // 哨兵具体指令结构体实例
sentry_cmd_t sentinel_decision;      // 哨兵自主决策通信指令整体结构体实例
sentry_get_info_t sentry_get_info;   // 哨兵获取信息结构体实例
eventdata_get_info_t event_get_info; // 场地事件数据获取结构体实例
void Sentry_Msg_Update(sentry_msg_t *msg)
{
   // 此处根据实际情况更新sentry_msg的字段值
   msg->revive_confirm = 0; // 确认复活
   msg->exch_revive = 0;    // 兑换立即复活

   msg->exch_bullet = 0;     // 设置发弹量
   msg->exch_bullet_cnt = 0; // 请求远程兑换发弹量次数
   msg->exch_blood_cnt = 0;  // 请求远程兑换血量次数
   msg->posture = 2;         // uart2_rx_message.posture;         // 设置姿态
   msg->buff_confirm = 0;    // 确认使能量机关进入正在激活状态
}

/* 32位指令如下：
bit 0：哨兵机器人是否确认复活(0表示哨兵机器人确认不复活)
bit 1: 哨兵机器人是否确认兑换立即复活(0表示哨兵机器人确认不兑换)
bit 2-12：哨兵将要兑换的发弹量值，开局为0，修改此值后，哨兵在补血点即可兑换允许发弹量,此值的变化需要单调递增
bit 13-16：哨兵远程兑换发弹量的请求次数，开局为0, 此值的变化需要单调递增且每次仅能增加1
bit 17-20：哨兵远程兑换血量的请求次数，开局为0，修改此值即可请求远程兑换血量,此值的变化需要单调递增且每次仅能增加1
bit 21-22：哨兵修改当前姿态指令，1为进攻姿态，2为防御姿态，3为移动姿态，默认为3；修改此值即可改变哨兵姿态
bit 23：哨兵机器人是否确认使能量机关进入正在激活状态，1为确认。默认为0
bit 24-31：保留位*/
void Sentry_Cmd_Pack(sentry_cmd_t *msg_cmd, sentry_msg_t *msg)
{
   uint32_t cmd = 0;

   // 位域打包逻辑(带位宽掩码保护)
   cmd |= (msg->revive_confirm & 0x01);
   cmd |= (msg->exch_revive & 0x01) << 1;
   cmd |= (msg->exch_bullet & 0x07FF) << 2;    // 11 bits
   cmd |= (msg->exch_bullet_cnt & 0x0F) << 13; // 4 bits
   cmd |= (msg->exch_blood_cnt & 0x0F) << 17;  // 4 bits
   cmd |= (msg->posture & 0x03) << 21;         // 2 bits
   cmd |= (msg->buff_confirm & 0x01) << 23;

   msg_cmd->sentry_cmd = cmd;
}

void Sentry_Send_Decision(sentry_cmd_t *msg_cmd, sentry_msg_t *msg)
{
   uint8_t temp_datalength = Interactive_Data_LEN_Head + Sentry_Msg_LEN; // 计算交互数据长度
   msg_cmd->header.SOF = 0xA5;
   msg_cmd->header.length = temp_datalength;
   msg_cmd->header.seq = seq;
   msg_cmd->header.crc8 = Get_CRC8_Check_Sum((uint8_t *)&msg_cmd, LEN_CRC8, 0xFF);
   msg_cmd->cmd_id = ID_student_interactive; // 机器人数据交互主ID
   msg_cmd->sub_id = Sentry_Msg_ID;          // 哨兵指令子ID
   msg_cmd->send_id = ui_self_id;
   msg_cmd->recv_id = 0x8080; // 给裁判系统
   // 哨兵补充结构体对所需指令设置--uint32_t sentry_cmd
   Sentry_Cmd_Pack(msg_cmd, msg); // 更新Pack据到sentry_cmd字段
   // msg_cmd->sentry_cmd = *(uint32_t *)&sentry_msg; //设置对应指令位
   msg_cmd->crc16 = Get_CRC16_Check_Sum((uint8_t *)&msg_cmd, LEN_HEADER + LEN_CMDID + temp_datalength, 0xFFFF);
   Referee_Send((uint8_t *)&sentinel_decision, LEN_HEADER + LEN_CMDID + temp_datalength + LEN_TAIL);
   seq++;
}


// 发送数据给裁判系统的函数，调用该函数即可将数据发送给裁判系统
void Sentrycmd_To_Referee()
{
   Sentry_Msg_Update(&sentry_msg); // 根据实际情况更新sentry_msg的字段值
   Sentry_Send_Decision(&sentinel_decision, &sentry_msg);
}


// 哨兵信息解包函数,从裁判系统接收的数据中提取哨兵相关信息并存储到sentry_get_info结构体实例中
void Sentry_Unpacked_Msg(sentry_get_info_t *get_info)
{
   // 1. 解包 sentry_info (32 bit)
   uint32_t info1 = referee_outer_info->SentryInfo.sentry_info;   // 从裁判系统数据结构中获取sentry_info字段
   uint16_t info2 = referee_outer_info->SentryInfo.sentry_info_2; // 从裁判系统数据结构中获取sentry_info_2字段

   get_info->exch_bullet = (info1 >> 0) & 0x07FF;
   get_info->remote_exch_bullet_cnt = (info1 >> 11) & 0x0F;
   get_info->remote_exch_blood_cnt = (info1 >> 15) & 0x0F;
   get_info->can_free_revive = (info1 >> 19) & 0x01;
   get_info->can_immediate_revive = (info1 >> 20) & 0x01;
   get_info->immediate_revive_cost = (info1 >> 21) & 0x03FF;

   // 2. 解包 sentry_info_2 (16 bit)
   get_info->out_of_combat = (info2 >> 0) & 0x01;
   get_info->remain_exch_bullet = (info2 >> 1) & 0x07FF;
   get_info->posture = (info2 >> 12) & 0x03;
   get_info->can_activate_buff = (info2 >> 14) & 0x01;
}

 //从裁判系统数据中解包出场地事件相关信息到event_get_info结构体
void EventData_Unpacked_Msg(eventdata_get_info_t *event_data)
{
   uint8_t raw_event = referee_outer_info->EventData.event_data;
   event_data->supply_status = (raw_event >> 0) & 0x01;

   // 能量机关状态
   event_data->s_buff_status = (raw_event >> 3) & 0x03;
   event_data->b_buff_status = (raw_event >> 5) & 0x03;

   // 高地占领状态
   event_data->center_upland_status = (raw_event >> 7) & 0x03;
   event_data->trapezoid_highland = (raw_event >> 9) & 0x03;

   // // 飞镖数据
   // event_data->dart_last_hit_time = (raw_event >> 11) & 0x01FF;
   // event_data->dart_last_hit_target = (raw_event >> 20) & 0x07;

   // 增益点占领状态
   event_data->fort_status = (raw_event >> 25) & 0x03;
   event_data->outpost_status = (raw_event >> 27) & 0x03;
   event_data->base_buff_zone = (raw_event >> 29) & 0x01;
}

void Referee_Data_Update() // 更新要传给上位机的裁判系统数据
{
   uart2_tx_message.game_time = referee_outer_info->GameState.stage_remain_time; // 获取当前阶段剩余时间
   uart2_tx_message.is_play = referee_outer_info->GameState.game_progress;       // 当前比赛阶段
   uart2_tx_message.own_hp = referee_outer_info->RobotPerformance.current_HP;    // 获取自身机器人HP
   uart2_tx_message.outpost_HP = referee_outer_info->GameRobotHP.robot_outpost_HP; // 获取前哨站HP
   uart2_tx_message.outpost_status = event_get_info.outpost_status; // 获取前哨站占领状态
   uart2_tx_message.fort_status = event_get_info.fort_status; // 获取堡垒占领状态
   uart2_tx_message.position_x = referee_outer_info->GameRobotPos.x; // 获取机器人位置x
   uart2_tx_message.position_y = referee_outer_info->GameRobotPos.y; // 获取机器人位置y 
   uart2_tx_message.tar_position_x = referee_outer_info->MapCommand.target_position_x; // 目标位置x（半自动模式使用）
   uart2_tx_message.tar_position_y = referee_outer_info->MapCommand.target_position_y; // 目标位置y（半自动模式使用）
}