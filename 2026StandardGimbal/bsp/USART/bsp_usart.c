/**
******************************************************************************
* @file    bsp_usart.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/
#include <stdlib.h>
#include <string.h>
#include "bsp_usart.h"
#include "rs485.h"

/* usart service instance, modules' info would be recoreded here using USART_Register() */
/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;
static USART_t *usart_instances[DEVICE_USART_CNT] = {NULL};

/**
 * @brief 启动串口服务,会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *
 * @todo 串口服务会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *       可能还要将此函数修改为extern,使得module可以控制串口的启停
 *
 * @param _instance instance owned by module,模块拥有的串口实例
 */
/**
 * @brief 启动串口服务,需要传入一个usart实例.一般用于lost callback的情况(使用串口的模块daemon)
 *
 * @param _instance
 */
void USART_Service_Init(USART_t *_instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(_instance->usart_handle,
                                 _instance->recv_buff,
                                 _instance->recv_buff_size);
    // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
    // 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
    // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
    __HAL_DMA_DISABLE_IT(_instance->usart_handle->hdmarx, DMA_IT_HT);
}

/**
 * @brief 注册一个串口实例,返回一个串口实例指针
 *
 * @param init_config 传入串口初始化结构体
 */
USART_t *USART_Register(usart_init_config_t *init_config)
{
    if (idx >= DEVICE_USART_CNT) // 超过最大实例数
    {
        while (1)
        {
            ;
        }
    }

    for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
    {
        if (usart_instances[i]->usart_handle == init_config->usart_handle)
        {
            while (1)
            {
                ;
            }
        }
    }

    USART_t *instance = (USART_t *)malloc(sizeof(USART_t));
    memset(instance, 0, sizeof(USART_t));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;
    instance->beat = 0;
    instance->lost_flag = 0;

    usart_instances[idx++] = instance;
    USART_Service_Init(instance);
    return instance;
}

/**
 * @brief 通过调用该函数可以发送一帧数据,需要传入一个usart实例,发送buff以及这一帧的长度
 * @note 在短时间内连续调用此接口,若采用IT/DMA会导致上一次的发送未完成而新的发送取消.
 * @note 若希望连续使用DMA/IT进行发送,请配合USARTIsReady()使用,或自行为你的module实现一个发送队列和任务.
 * @todo 是否考虑为USARTInstance增加发送队列以进行连续发送?
 *
 * @param _instance 串口实例
 * @param send_buf 待发送数据的buffer
 * @param send_size how many bytes to send
 */
/* @todo 当前仅进行了形式上的封装,后续要进一步考虑是否将module的行为与bsp完全分离 */
void USART_Send(USART_t *_instance,
                uint8_t *send_buf,
                uint16_t send_size,
                usart_transfer_e mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        while (1)
            ; // illegal mode! check your code context! 检查定义instance的代码上下文,可能出现指针越界
              //			break;
    }
}

/**
 * @brief 判断串口是否准备好,用于连续或异步的IT/DMA发送
 *
 * @param _instance 要判断的串口实例
 * @return uint8_t ready 1, busy 0
 */
/* 串口发送时,gstate会被设为BUSY_TX */
uint8_t USART_Is_Ready(USART_t *_instance)
{
    if (_instance->usart_handle->gState | HAL_UART_STATE_BUSY_TX)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

uint8_t USART_Error_Lost(USART_t *_instance)
{
    uint8_t error_cnt;

    error_cnt = 0;

    if (_instance == NULL)
    {
        for (size_t i = 0; i < idx; ++i)
        {
            usart_instances[i]->beat++;

            if (usart_instances[i]->beat >= 500)
            {
                usart_instances[i]->lost_flag = 1;
            }

            if (usart_instances[i]->lost_flag == 1)
            {
                error_cnt++;
            }
        }
    }
    else
    {
        _instance->beat++;

        if (_instance->beat >= 500)
        {
            _instance->lost_flag = 1;
        }

        if (_instance->lost_flag == 1)
        {
            error_cnt = 1;
        }
    }

    return error_cnt;
}

/**
 * @brief 每次dma/idle中断发生时，都会调用此函数.对于每个uart实例会调用对应的回调进行进一步的处理
 *        例如:视觉协议解析/遥控器解析/裁判系统解析
 *
 * @note  通过__HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT)关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
 *        这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
 *        我们只希望处理，因此直接关闭DMA半传输中断第一种和第三种情况
 *
 * @param huart 发生中断的串口
 * @param Size 此次接收到的总数居量,暂时没用
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    { // find the instance which is being handled
        if (huart == usart_instances[i]->usart_handle)
        { // call the callback function if it is not NULL
            usart_instances[i]->beat = 0;
            if (usart_instances[i]->module_callback != NULL)
            {
                usart_instances[i]->current_size = Size;
                usart_instances[i]->module_callback();
                memset(usart_instances[i]->recv_buff, 0, Size); // 接收结束后清空buffer,对于变长数据是必要的
            }
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instances[i]->usart_handle,
                                         usart_instances[i]->recv_buff,
                                         usart_instances[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instances[i]->usart_handle->hdmarx, DMA_IT_HT);
            return; // break the loop
        }
    }
}

/**
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instances[i]->usart_handle)
        {
            HAL_UART_DMAStop(usart_instances[i]->usart_handle);
            // 检查错误标志位
            if (__HAL_UART_GET_FLAG(usart_instances[i]->usart_handle, UART_FLAG_NE)) // 噪声错误标志位
            {
                __HAL_UART_CLEAR_NEFLAG(usart_instances[i]->usart_handle); // 清除
            }
            if (__HAL_UART_GET_FLAG(usart_instances[i]->usart_handle, UART_FLAG_FE))
            {
                __HAL_UART_CLEAR_FEFLAG(usart_instances[i]->usart_handle);
            }
            if (__HAL_UART_GET_FLAG(usart_instances[i]->usart_handle, UART_FLAG_PE))
            {
                __HAL_UART_CLEAR_PEFLAG(usart_instances[i]->usart_handle); // 清除奇偶校验错误标志位
            }
            if (__HAL_UART_GET_FLAG(usart_instances[i]->usart_handle, UART_FLAG_ORE)) // 溢出错误标志位
            {
                __HAL_UART_CLEAR_OREFLAG(usart_instances[i]->usart_handle);
            }
            HAL_UART_DeInit(usart_instances[i]->usart_handle); // 关闭串口
            HAL_UART_Init(usart_instances[i]->usart_handle);   // 重新初始化串口
            HAL_UARTEx_ReceiveToIdle_DMA(usart_instances[i]->usart_handle,
                                         usart_instances[i]->recv_buff,
                                         usart_instances[i]->recv_buff_size);
            __HAL_DMA_DISABLE_IT(usart_instances[i]->usart_handle->hdmarx, DMA_IT_HT);
            return;
        }
    }
    if (huart == &huart2)
    {
    //     if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)) // 噪声错误标志位
    //     {
    //         __HAL_UART_CLEAR_NEFLAG(huart); // 清除
    //     }
    //     if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
    //     {
    //         __HAL_UART_CLEAR_FEFLAG(huart);
    //     }
    //     if (__HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
    //     {
    //         __HAL_UART_CLEAR_PEFLAG(huart); // 清除奇偶校验错误标志位
    //     }
    //     if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) // 溢出错误标志位
    //     {
    //         __HAL_UART_CLEAR_OREFLAG(huart);
    //     }
    //     HAL_UART_DeInit(huart); // 关闭串口
    //     HAL_UART_Init(huart);   // 重新初始化串口
    //     HAL_UART_Receive_IT(&huart2, &uart2_current_byte, 1);
    // }
    // if (huart == &huart2)
    // {
    //     uart2_buffer_length = 0;
    //     uart2_status = ready_to_receive;
        HAL_UART_Receive_IT(&huart2, &uart2_current_byte, 1);
    }
}
