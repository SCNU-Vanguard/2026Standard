#include "rs485.h"
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>

#define FRAME_HEADER 0xA5 // 帧头
#define FRAME_TAILER 0x5A // 帧尾

Tx_packed_t uart2_tx_message; // 发送数据结构体
Rx_packed_t uart2_rx_message; // 接收数据结构体

uint8_t uart2_receive_buffer[sizeof(Rx_packed_t) * 3]; // 接收缓冲区
uint8_t uart2_transmit_buffer[sizeof(Tx_packed_t)];    // 发送缓冲区

uint8_t ready_to_transmit = 0; // 发送就绪标志位
uint8_t ready_to_receive = 1;  // 接收就绪标志位

uint8_t uart2_status = 0;       // 状态标志位
uint32_t last_uart2_uwTick = 0; // 上次发送时间戳

uint16_t uart2_buffer_length = 0;
uint8_t uart2_current_byte = 0;

float cnttttt = 0;
uint8_t rs485_status = 0;

static void uart2_append_byte(uint8_t byte)
{
    if (uart2_buffer_length >= sizeof(uart2_receive_buffer))
    {
        memmove(uart2_receive_buffer, uart2_receive_buffer + 1, sizeof(uart2_receive_buffer) - 1);
        uart2_buffer_length = sizeof(uart2_receive_buffer) - 1;
    }
    uart2_receive_buffer[uart2_buffer_length++] = byte;
}

static uint8_t uart2_extract_frame(Rx_packed_t *frame)
{
    while (uart2_buffer_length >= sizeof(Rx_packed_t))
    {
        if (uart2_receive_buffer[0] != FRAME_HEADER)
        {
            memmove(uart2_receive_buffer, uart2_receive_buffer + 1, uart2_buffer_length - 1);
            uart2_buffer_length--;
            continue;
        }

        if (uart2_receive_buffer[sizeof(Rx_packed_t) - 2] != FRAME_TAILER)
        {
            memmove(uart2_receive_buffer, uart2_receive_buffer + 1, uart2_buffer_length - 1);
            uart2_buffer_length--;
            continue;
        }

        uint8_t received_check_sum = 0;
        for (uint16_t i = 0; i < sizeof(Rx_packed_t) - 1; i++)
        {
            received_check_sum += uart2_receive_buffer[i];
        }

        if (received_check_sum != uart2_receive_buffer[sizeof(Rx_packed_t) - 1])
        {
            memmove(uart2_receive_buffer, uart2_receive_buffer + 1, uart2_buffer_length - 1);
            uart2_buffer_length--;
            continue;
        }

        memcpy(frame, uart2_receive_buffer, sizeof(Rx_packed_t));
        memmove(uart2_receive_buffer, uart2_receive_buffer + sizeof(Rx_packed_t), uart2_buffer_length - sizeof(Rx_packed_t));
        uart2_buffer_length -= sizeof(Rx_packed_t);
        return 1;
    }

    return 0;
}

// 记得启用接收中断HAL_UART_Receive_IT(&huart2, &uart2_current_byte, 1);

void uart2_send_data(Tx_packed_t *data_p) // 发送数据函数
{
    data_p->frame_header = FRAME_HEADER;
    data_p->frame_tailer = FRAME_TAILER;
    data_p->check_sum = 0;
    for (uint8_t i = 0; i < sizeof(Tx_packed_t) - 1; i++) // 计算校验和，这里校验和放在帧尾后面
    {
        data_p->check_sum += ((uint8_t *)data_p)[i];
    }
    memcpy(uart2_transmit_buffer, data_p, sizeof(Tx_packed_t));
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_UART_Transmit_IT(&huart2, uart2_transmit_buffer, sizeof(Tx_packed_t));
}

void uart2_transmit_control(void) // 发送控制函数，发送时直接调用这个函数即可
{
    if (uart2_status == ready_to_transmit)
    {
        uart2_send_data(&uart2_tx_message);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // 接收回调函数
{
    if (huart == &huart2)
    {
        uart2_append_byte(uart2_current_byte);

        if (uart2_extract_frame(&uart2_rx_message))
        {
            last_uart2_uwTick = uwTick;
            uart2_status = ready_to_transmit;
            uart2_transmit_control();
            cnttttt++;
        }
        HAL_UART_Receive_IT(&huart2, &uart2_current_byte, 1);
    }
}

void uart2_online_check(void)
{
    if (uwTick - last_uart2_uwTick > 100)
    {
        uart2_rx_message.chassis_mode = 0;
        uart2_rx_message.target_x_speed = 0;
        uart2_rx_message.target_y_speed = 0;
        uart2_rx_message.target_omega_speed = 0;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) // 发送回调函数
{
    if (huart == &huart2)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
        uart2_status = ready_to_receive;
    }
}

// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) // 错误回调函数
// {
//     memset(uart2_receive_buffer, 0, sizeof(uart2_receive_buffer));
//     uart2_buffer_length = 0;
//     memset(&uart2_rx_message, 0, sizeof(Rx_packed_t));
//     HAL_UART_Receive_IT(&huart2, &uart2_current_byte, 1);
// }
