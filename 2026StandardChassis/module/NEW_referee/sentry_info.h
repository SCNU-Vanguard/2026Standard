#ifndef SENTRY_INFO_H   
#define SENTRY_INFO_H

#include "main.h"

#include "stm32h7xx_hal.h"

#include "usart.h"
#include "bsp_usart.h"


typedef struct
{
    uint32_t revive_confirm : 1;   // bit 0：确认复活
    uint32_t exch_revive : 1;      // bit 1：确认兑换立即复活
    uint32_t exch_bullet : 11;     // bit 2-12：兑换发弹量值 (单调递增)
    uint32_t exch_bullet_cnt : 4;  // bit 13-16：远程兑换发弹量次数 (单调递增)
    uint32_t exch_blood_cnt : 4;   // bit 17-20：远程兑换血量次数 (单调递增)
    uint32_t posture : 2;          // bit 21-22：姿态 (1进攻, 2防御, 3移动)
    uint32_t buff_confirm : 1;     // bit 23：确认使能量机关进入正在激活状态
    uint32_t reserved : 8;         // bit 24-31：保留位
}__attribute__((packed))sentry_msg_t;

typedef struct {
    uint8_t SOF;
    uint16_t length;
    uint8_t seq, crc8;
}__attribute__((packed))referee_frame_header_t;

typedef struct {
    referee_frame_header_t header;
    uint16_t cmd_id;   //主id
    uint16_t sub_id;   //子id
    uint16_t send_id, recv_id;  //发送者ID和接收者ID，裁判系统会根据这个字段来区分不同的机器人或模块
    uint32_t sentry_cmd;
    uint16_t crc16;
}__attribute__((packed))sentry_cmd_t;

typedef struct
{
    //解析自 sentry_info (32 bits)
    uint16_t exch_bullet;          // bit 0-10 (11位): 成功兑换的允许发弹量
    uint8_t  remote_exch_bullet_cnt;    // bit 11-14 (4位): 成功远程兑换发弹量次数
    uint8_t  remote_exch_blood_cnt;     // bit 15-18 (4位): 成功远程兑换血量次数
    uint8_t  can_free_revive;           // bit 19 (1位): 是否可以免费复活 (1可/0否)
    uint8_t  can_immediate_revive;      // bit 20 (1位): 是否可以兑换立即复活 (1可/0否)
    uint16_t immediate_revive_cost;     // bit 21-30 (10位): 兑换立即复活需要的金币数

    //解析自 sentry_info_2 (16 bits)
    uint8_t  out_of_combat;          // bit 0 (1位): 是否处于脱战状态 (1是/0否)
    uint16_t remain_exch_bullet;        // bit 1-11 (11位): 队伍 17mm 允许发弹量剩余可兑换数
    uint8_t  posture;                   // bit 12-13 (2位): 哨兵当前姿态 (1进攻, 2防御, 3移动)
    uint8_t  can_activate_buff;         // bit 14 (1位): 己方能量机关是否能进入激活状态
} sentry_get_info_t;


typedef struct
{
    uint8_t supply_status; // bit 0-2: 补给区占领状态（只有bit0有效，其他两位数据无作用不解析）
    uint8_t s_buff_status;  // bit 3-4: 小能量机关激活状态（0未激活，1已激活，2激活中）
    uint8_t b_buff_status;  // bit 5-6: 大能量机关激活状态（0未激活，1已激活，2激活中）
    uint8_t center_upland_status; // bit 7-8: 己方中心高地占领状态（1己方占领，2对方占领）
    uint8_t trapezoid_highland;       // bit 9-10 (2位): 己方梯形高地的占领状态 (1为已占领)

    // --- 飞镖数据 ---
    uint16_t dart_last_hit_time;      // bit 11-19 (9位): 对方飞镖最后一次击中己方前哨站或基地的时间 (0-420)
    uint8_t  dart_last_hit_target;    // bit 20-22 (3位): 对方飞镖最后一次击中具体目标 (0-5)

    // --- 增益点占领状态 ---
    uint8_t center_status;         // bit 23-24 (2位): 中心增益点的占领状态 (0-3, 仅 RMUL)
    uint8_t fort_status;           // bit 25-26 (2位): 己方堡垒增益点的占领状态 (0-3)
    uint8_t outpost_status;        // bit 27-28 (2位): 己方前哨站增益点的占领状态 (0-2)
    uint8_t base_status;           // bit 29    (1位): 己方基地增益点的占领状态 (1为已占领)

    // bit 30-31 为保留位，无需解析
} eventdata_get_info_t;

extern USART_t *referee_usart_instance;

extern uint8_t seq;

extern sentry_msg_t sentry_msg; // 哨兵指令结构体实例
extern sentry_get_info_t sentry_get_info; // 哨兵获取信息结构体实例
extern eventdata_get_info_t event_get_info; // 场地事件数据获取结构体实例

extern void Sentrycmd_To_Referee();
extern void Sentry_Send_Decision(sentry_msg_t *msg);
extern void Referee_Data_Update();
extern void Sentry_Unpacked_Msg(sentry_get_info_t *get_info);
extern void EventData_Unpacked_Msg(eventdata_get_info_t *event_data);
#endif //SENTRY_INFO_H