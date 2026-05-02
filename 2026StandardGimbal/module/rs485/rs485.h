#ifndef __RS485_H
#define __RS485_H

#include "main.h"

#define FRAME_HEADER 0xA5
#define FRAME_TAILER 0x5A

typedef struct
{
    uint8_t frame_header;
    uint8_t chassis_mode;
    uint8_t shoot_mode;
    float target_angle_yaw;
    float target_x_speed;
    float target_y_speed;
    float target_omega_speed;
    float INS_yaw;
    float INS_Gyro_Z;
    uint8_t posture;
    uint8_t frame_tailer;
    uint8_t check_sum;
} __attribute__((packed)) Tx_packed_t;

typedef struct
{
    uint8_t frame_header;
    float gimbal_angle_yaw_motor2imu;
    float yaw_vel;
    uint8_t is_play;      //比赛阶段
    uint16_t game_time;   // 比赛时间
    float positon_x;      //机器人自身位置
    float positon_y;
    float tar_positon_x;  //目标地点位置（半自动模式使用）
    float tar_positon_y;
    uint16_t own_hp;   // 机器人自身血量
    uint16_t outpost_HP; //己方前哨站血量
    uint8_t outpost_status;    //前哨站占领状态
    uint8_t fort_status;    //堡垒占领状态
    uint8_t frame_tailer;
    uint8_t check_sum;
} __attribute__((packed)) Rx_packed_t;

extern Tx_packed_t uart2_tx_message;
extern Rx_packed_t uart2_rx_message;

extern uint8_t uart2_receive_buffer[sizeof(Rx_packed_t) * 3];
extern uint8_t uart2_transmit_buffer[sizeof(Tx_packed_t)];

extern uint8_t ready_to_transmit;
extern uint8_t ready_to_receive;

extern uint8_t uart2_status;
extern uint32_t last_uart2_uwTick;
extern uint8_t uart2_current_byte;
extern uint16_t uart2_buffer_length;
extern uint8_t rs485_status;

void uart2_send_data(Tx_packed_t *data_p);
void uart2_transmit_control(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void uart2_online_check(void);

#endif
