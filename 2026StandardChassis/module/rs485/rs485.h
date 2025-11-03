#ifndef __RS485_H
#define __RS485_H

#include "main.h"

#define FRAME_HEADER 0xA5
#define FRAME_TAILER 0x5A

typedef struct
{
    uint8_t frame_header;
    float chassis_omega_speed;
    uint8_t frame_tailer;
    uint8_t check_sum;
} __attribute__((packed)) Tx_packed_t;

typedef struct
{
    uint8_t frame_header;
    uint8_t chassis_mode;
    int16_t delta_target_angle_yaw;
    int16_t target_x_speed;
    int16_t target_y_speed;
    float target_omega_speed;
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
extern uint8_t rs485_status;

void uart2_send_data(Tx_packed_t *data_p);
void uart2_transmit_control(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void uart2_online_check(void);

#endif
