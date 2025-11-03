/**
******************************************************************************
* @file    DM_motor.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/
#include "DM_motor.h"

#include "bsp_dwt.h"

static uint8_t idx;
static DM_motor_t *dm_motor_instances[DM_MOTOR_CNT];

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

float Hex_To_Float(uint32_t *Byte, int num) // 十六进制到浮点数
{
    return *((float *)Byte);
}

uint32_t Float_To_Hex(float HEX) // 浮点数到十六进制转换
{
    return *(uint32_t *)&HEX;
}

/**
 ************************************************************************
 * @brief:      	float_to_uint: 浮点数转换为无符号整数函数
 * @param[in]:   x_float:	待转换的浮点数
 * @param[in]:   x_min:		范围最小值
 * @param[in]:   x_max:		范围最大值
 * @param[in]:   bits: 		目标无符号整数的位数
 * @retval:     	无符号整数结果
 * @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
 ************************************************************************
 **/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 ************************************************************************
 * @brief:      	uint_to_float: 无符号整数转换为浮点数函数
 * @param[in]:   x_int: 待转换的无符号整数
 * @param[in]:   x_min: 范围最小值
 * @param[in]:   x_max: 范围最大值
 * @param[in]:   bits:  无符号整数的位数
 * @retval:     	浮点数结果
 * @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
 ************************************************************************
 **/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DM_Motor_Set_Mode(DM_motor_mode_e cmd, DM_motor_t *motor)
{
    memset(motor->motor_can_instance->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instance->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CAN_Transmit(motor->motor_can_instance, 10);
}

void DM_Motor_Set_Zeropoint(DM_motor_t *motor)
{
    DM_Motor_Set_Mode(DM_CMD_ZERO_POSITION, motor);
}

void DM_Motor_Start(DM_motor_t *motor)
{
    if (motor == NULL)
    {
        for (size_t i = 0; i < idx; ++i)
        {
            dm_motor_instances[i]->motor_state_flag = MOTOR_ENABLE;
        }
    }
    else
    {
        motor->motor_state_flag = MOTOR_ENABLE;
    }
}

void DM_Motor_Stop(DM_motor_t *motor)
{
    if (motor == NULL)
    {
        for (size_t i = 0; i < idx; ++i)
        {
            dm_motor_instances[i]->motor_state_flag = MOTOR_DISABLE;
        }
    }
    else
    {
        motor->motor_state_flag = MOTOR_DISABLE;
    }
}

void DM_Motor_ENABLE(DM_motor_t *motor)
{
    if (motor == NULL)
    {
        for (size_t i = 0; i < idx; ++i)
        {
            DM_Motor_Set_Mode(DM_CMD_ENABLE_MODE, dm_motor_instances[i]);
        }
    }
    else
    {
        DM_Motor_Set_Mode(DM_CMD_ENABLE_MODE, motor);
    }
}

void DM_Motor_DISABLE(DM_motor_t *motor)
{
    if (motor == NULL)
    {
        for (size_t i = 0; i < idx; ++i)
        {
            DM_Motor_Set_Mode(DM_CMD_DISABLE_MODE, dm_motor_instances[i]);
        }
    }
    else
    {
        DM_Motor_Set_Mode(DM_CMD_DISABLE_MODE, motor);
    }
}

void DM_Motor_Clear_Error(DM_motor_t *motor)
{
    DM_Motor_Set_Mode(DM_CMD_CLEAR_ERROR, motor);
}

void DM_Motor_SetTar(DM_motor_t *motor, float val)
{
    motor->motor_controller.pid_ref = val;
}

// MIT 模式是为了兼容原版 MIT 模式所设计，可以在实现无缝切换的同时，能够灵活设
// 定控制范围（P_MAX,V_MAX，T_MAX），电调将接收到的 CAN 数据转化成控制变量进行
// 运算得到扭矩值作为电流环的电流给定，电流环根据其调节规律最终达到给定的扭矩电流。
// 根据 MIT 模式可以衍生出多种控制模式，如 kp=0,kd 不为 0 时，给定 v_des 即可实现匀
// 速转动;kp=0,kd=0，给定 t_ff 即可实现给定扭矩输出。
// 注意：对位置进行控制时，kd 不能赋 0，否则会造成电机震荡，甚至失控。
// 1. 当 kp=0，kd≠0 时，给定 v_des 即可实现匀速转动。匀速转动过程中存在静差，另外 kd 不宜过大， kd 过大时会引起震荡。
// 2. 当 kp=0，kd=0 时，给定 t_ff 即可实现给定扭矩输出。在该情况下，电机会持续输出一个恒定力矩。但是当电机空转或负载较小时，如果给定 t_ff 较大，电机会持续加速，直到加速到最大速度，这时也仍然达不到目标力矩 t_ff。
// 3. 当 kp≠0，kd=0 时，会引起震荡。即对位置进行控制时，kd 不能赋0，否则会造成电机震荡，甚至失控。
// 4. 当 kp≠0，kd≠0 时，有多种情况，这里下面简单介绍两种情况。
//
//- 当期望位置 p_des 为常量，期望速度 v_des 为0时，可实现定点控制，在这个过程中，实际位置趋近于p_des，实际速度 dθm 趋近于0。
//- 当p_des是随时间变化的连续可导函数时，同时 v_des 是 p_des 的导数，可实现位置跟踪和速度跟踪，即按照期望速度旋转期望角度。
/**
 ************************************************************************
 * @brief:      	mit_ctrl: MIT模式下的电机控制函数
 * @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
 * @param[in]:   motor_id:	电机ID，指定目标电机
 * @param[in]:   pos:			位置给定值
 * @param[in]:   vel:			速度给定值
 * @param[in]:   kp:				位置比例系数
 * @param[in]:   kd:				位置微分系数
 * @param[in]:   torq:			转矩给定值
 * @retval:     	void
 * @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
 ************************************************************************
 **/
void DM_MIT_Ctrl(DM_motor_t *motor,
                 float pos,
                 float vel,
                 float kp,
                 float kd,
                 float torq)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    pos_tmp = float_to_uint(pos, DM_P_MIN, DM_P_MAX, 16);
    vel_tmp = float_to_uint(vel, DM_V_MIN, DM_V_MAX, 12);
    kp_tmp = float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    kd_tmp = float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);
    tor_tmp = float_to_uint(torq, DM_T_MIN, DM_T_MAX, 12);

    motor->motor_can_instance->tx_buff[0] = (pos_tmp >> 8);
    motor->motor_can_instance->tx_buff[1] = pos_tmp;
    motor->motor_can_instance->tx_buff[2] = (vel_tmp >> 4);
    motor->motor_can_instance->tx_buff[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    motor->motor_can_instance->tx_buff[4] = kp_tmp;
    motor->motor_can_instance->tx_buff[5] = (kd_tmp >> 4);
    motor->motor_can_instance->tx_buff[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    motor->motor_can_instance->tx_buff[7] = tor_tmp;

    motor->motor_can_instance->tx_header.Identifier = motor->dm_tx_id + MIT_MODE;
    CAN_Transmit(motor->motor_can_instance, 1);
}

// 位置串级模式是采用三环串联控制的模式，位置环作为最外环，其输出作为速度环的给
// 定，而速度环的输出作为内环电流环的给定，用以控制实际的电流输出
// p_des 为控制的目标位置，v_des 是用来限定运动过程中的最大绝对速度值。
// 位置串级模式如使用调试助手推荐的控制参数控制，可以达到较好的控制精度，控制过
// 程相对柔顺，但响应时间相对较长。可配置的相关参数除 v_des 外，另有加/减速度进行设
// 定，如控制过程中产生额外的震荡可提高加/减速度。
// 注意：p_des，v_des 单位分别为 rad 和 rad/s，数据类型为 float，阻尼因子必须设置为
// 非 0 的正数，可参考速度模式的注意事项。
/**
 ************************************************************************
 * @brief:      	pos_speed_ctrl: 位置速度控制函数
 * @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
 * @param[in]:   motor_id:	电机ID，指定目标电机
 * @param[in]:   vel:			速度给定值
 * @retval:     	void
 * @details:    	通过CAN总线向电机发送位置速度控制命令
 ************************************************************************
 **/
void DM_Pos_Speed_Ctrl(DM_motor_t *motor, float pos, float vel)
{
    uint8_t *pbuf, *vbuf;

    pbuf = (uint8_t *)&pos;
    vbuf = (uint8_t *)&vel;

    motor->motor_can_instance->tx_buff[0] = *pbuf;
    motor->motor_can_instance->tx_buff[1] = *(pbuf + 1);
    motor->motor_can_instance->tx_buff[2] = *(pbuf + 2);
    motor->motor_can_instance->tx_buff[3] = *(pbuf + 3);

    motor->motor_can_instance->tx_buff[4] = *vbuf;
    motor->motor_can_instance->tx_buff[5] = *(vbuf + 1);
    motor->motor_can_instance->tx_buff[6] = *(vbuf + 2);
    motor->motor_can_instance->tx_buff[7] = *(vbuf + 3);

    motor->motor_can_instance->tx_header.Identifier = motor->dm_tx_id + POS_MODE;
    CAN_Transmit(motor->motor_can_instance, 1);
}

// 速度模式能让电机稳定运行在设定的速度
// 注意：v_des 单位为 rad/s，数据类型为 float，如需使用调试助手自动计算参数，则需要
// 设置阻尼因子为非 0 正数，通常情况下取值在 2.0~10.0，过小的阻尼因子会带来速度的震荡
// 以及较大的过冲，过大的阻尼因子则会带来较长的上升时间，推荐的设定值为 4.0。
/**
 ************************************************************************
 * @brief:      	speed_ctrl: 速度控制函数
 * @param[in]:   hcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
 * @param[in]:   motor_id: 电机ID，指定目标电机
 * @param[in]:   vel: 			速度给定值
 * @retval:     	void
 * @details:    	通过CAN总线向电机发送速度控制命令
 ************************************************************************
 **/
void DM_Speed_Ctrl(DM_motor_t *motor, float vel)
{
    uint8_t *vbuf;

    vbuf = (uint8_t *)&vel;

    motor->motor_can_instance->tx_buff[0] = *vbuf;
    motor->motor_can_instance->tx_buff[1] = *(vbuf + 1);
    motor->motor_can_instance->tx_buff[2] = *(vbuf + 2);
    motor->motor_can_instance->tx_buff[3] = *(vbuf + 3);

    motor->motor_can_instance->tx_header.Identifier = motor->dm_tx_id + SPEED_MODE;
    CAN_Transmit(motor->motor_can_instance, 1);
}

void DM_DM124_Ctrl(DM_motor_t *motor, float pid_ref)
{
    int16_t pid_ref_int;
    pid_ref_int = (int16_t)pid_ref;

    memset(motor->motor_can_instance->tx_buff, 0, 8);

    // motor->motor_can_instance->tx_buff[0] = (uint8_t)(pid_ref_int >> 8);     // 低八位
    // motor->motor_can_instance->tx_buff[1] = (uint8_t)(pid_ref_int & 0x00ff); // 高八位
    motor->motor_can_instance->tx_buff[0] = (uint8_t)(pid_ref_int & 0x00ff); // 高八位
    motor->motor_can_instance->tx_buff[1] = (uint8_t)(pid_ref_int >> 8);     // 低八位

    motor->motor_can_instance->tx_header.Identifier = 0x3fe;

    CAN_Transmit(motor->motor_can_instance, 0);
}

static void DM_Motor_Decode(CAN_instance_t *motor_can)
{
    float velocity = 0.0f;

    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DM_motor_t *motor = (DM_motor_t *)motor_can->id;
    DM_motor_callback_t *receive_data = &(motor->receive_data); // 将can实例中保存的id转换成电机实例的指针

    velocity = receive_data->velocity;

    Supervisor_Reload(motor->supervisor);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    if (motor->error_code & DM_MOTOR_LOST_ERROR)
    {
        motor->error_code &= ~(DM_MOTOR_LOST_ERROR);
    }

    receive_data->last_position = receive_data->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    receive_data->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    receive_data->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);
    receive_data->velocity = (1 - SPEED_RAMP_COEF) * velocity + SPEED_RAMP_COEF * receive_data->velocity;

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    receive_data->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    receive_data->t_mos = (float)rxbuff[6];
    receive_data->t_rotor = (float)rxbuff[7];

    // receive_data->last_position = receive_data->position;
    // receive_data->position = ((uint16_t) rxbuff[0]) << 8 | rxbuff[1];
    // receive_data->velocity = (float) ((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    // receive_data->torque = (float) ((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    // receive_data->t_mos = rxbuff[6];
    // receive_data->t_rotor = rxbuff[7];
}

static void DM_Motor_Lost_Callback(void *motor_ptr)
{
    DM_motor_t *motor = (DM_motor_t *)motor_ptr;
    motor->error_code |= DM_MOTOR_LOST_ERROR;
}

// 经典CAN
// CAN_TxHeaderTypeDef DM_txheader = {
//     .ExtId = 0x0 ,
//     .IDE = CAN_ID_STD ,
//     .RTR = CAN_RTR_DATA ,
//     .DLC = 8 , };

// CAN_RxHeaderTypeDef DM_rxheader = {
//     .ExtId = 0x0 ,
//     .IDE = CAN_ID_STD ,
//     .RTR = CAN_RTR_DATA ,
//     .DLC = 8 , };

DM_motor_t *DM_Motor_Init(motor_init_config_t *config)
{
    DM_motor_t *instance = (DM_motor_t *)malloc(sizeof(DM_motor_t));
    memset(instance, 0, sizeof(DM_motor_t));

    if (instance == NULL)
    {
        return NULL;
    }

    instance->dm_mode = MIT_MODE;
    instance->motor_feedback = DM_MOTOR_ABSOLUTE;

    instance->motor_type = config->motor_type;
    instance->motor_settings = config->controller_setting_init_config;

    instance->motor_controller.torque_PID = PID_Init(
        config->controller_param_init_config.torque_PID);
    instance->motor_controller.speed_PID = PID_Init(
        config->controller_param_init_config.speed_PID);
    instance->motor_controller.angle_PID = PID_Init(
        config->controller_param_init_config.angle_PID);

    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.torque_feedforward_ptr = config->controller_param_init_config.torque_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

    instance->dm_tx_id = config->can_init_config.tx_id;
    instance->dm_rx_id = config->can_init_config.rx_id;

    config->can_init_config.can_module_callback = DM_Motor_Decode;
    config->can_init_config.id = instance;
    instance->motor_can_instance = CAN_Register(&config->can_init_config);

    // 注册守护线程
    supervisor_init_config_t supervisor_config = {
        .handler_callback = DM_Motor_Lost_Callback,
        .owner_id = instance,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    instance->supervisor = Supervisor_Register(&supervisor_config);

    instance->error_code = DM_ERROR_NONE;

    DWT_GetDeltaT(&instance->feed_cnt);

    DM_Motor_ENABLE(instance);
    DWT_Delay(0.1);
    dm_motor_instances[idx++] = instance;

    return instance;
}

uint8_t DM_Motor_Error_Judge(DM_motor_t *motor)
{
    uint8_t error_cnt;

    error_cnt = 0;

    if (motor == NULL)
    {
        for (size_t i = 0; i < idx; ++i)
        {
            if (dm_motor_instances[i]->receive_data.state != 0)
            {
                error_cnt++;
            }

            if ((dm_motor_instances[i]->receive_data.torque > 5.0f) || (dm_motor_instances[i]->receive_data.velocity > 5.0f))
            {
                error_cnt++;
                dm_motor_instances[i]->error_code |= DM_MOTOR_SUPERLOAD_ERROR;
            }
            else
            {
                dm_motor_instances[i]->error_code &= ~(DM_MOTOR_SUPERLOAD_ERROR);
            }
        }
    }
    else
    {
        if (motor->receive_data.state != 0)
        {
            error_cnt = 1;
        }

        if ((motor->receive_data.torque > 5.0f) || (motor->receive_data.velocity > 5.0f))
        {
            error_cnt = 1;
            motor->error_code |= DM_MOTOR_SUPERLOAD_ERROR;
        }
        else
        {
            motor->error_code &= ~(DM_MOTOR_SUPERLOAD_ERROR);
        }
    }

    return error_cnt;
}

/**
 * @brief 该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
// 为所有电机实例计算三环PID,发送控制报文
void DM_Motor_Control(void)
{
    // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    DM_motor_t *motor;
    motor_control_setting_t *motor_setting; // 电机控制参数
    motor_controller_t *motor_controller;   // 电机控制器
    DM_motor_callback_t *receive_data;      // 电机测量值
    float pid_fab, pid_ref;                 // 电机PID测量值和设定值

    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i)
    { // 减小访存开销,先保存指针引用
        motor = dm_motor_instances[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        receive_data = &motor->receive_data;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        // 多环目标值是上环输出为下环输入

        /* ------------------------------digital_pid------------------------------------*/
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        {
            pid_ref *= -1; // 目标值设置反转
        }

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        // dm_diff的值需要自己另外做处理，比如使其指向视觉发送的目标装甲板的偏差值
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && (motor_setting->outer_loop_type == ANGLE_LOOP))
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
            {
                pid_fab = *motor_controller->other_angle_feedback_ptr;
            }
            else
            {
                if (motor->motor_feedback == DM_MOTOR_ABSOLUTE)
                {
                    pid_fab = motor->receive_data.position;
                }
                else if (motor->motor_feedback == DM_MOTOR_DIFF)
                {
                    pid_fab = motor->receive_data.dm_diff;
                }
            }
            // 更新pid_ref进入下一个环
            pid_ref = PID_Position(motor_controller->angle_PID,
                                   pid_fab,
                                   pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor->motor_feedback == DM_MOTOR_ABSOLUTE)
            {
                pid_fab = motor->receive_data.position;
            }
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
            {
                pid_ref += *motor_controller->speed_feedforward_ptr;
            }

            if (motor_setting->speed_feedback_source == OTHER_FEED)
            {
                pid_fab = *motor_controller->other_speed_feedback_ptr;
            }

            // 更新pid_ref进入下一个环
            pid_ref = PID_Increment(motor_controller->speed_PID,
                                    receive_data->velocity,
                                    pid_ref);
        }

        // 计算扭矩环,目前只要启用了扭矩环就计算,不管外层闭环是什么,并且扭矩只有电机自身传感器的反馈
        if (motor_setting->feedforward_flag & TORQUE_FEEDFORWARD)
        {
            pid_ref += *motor_controller->torque_feedforward_ptr;
        }
        if (motor_setting->close_loop_type & TORQUE_LOOP)
        {
            pid_ref = PID_Position(motor_controller->torque_PID,
                                   receive_data->torque,
                                   pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
        {
            pid_ref *= -1; // 输出值设置反转
        }

        /* ------------------------------digital_pid------------------------------------*/

        /* ------------------------------handler------------------------------------*/
        // 获取最终输出
        // 判断模式再发送
        if (motor->dm_mode == SPEED_MODE)
        {
            // 若该电机处于停止状态,直接将buff置零
            if (motor->motor_state_flag == MOTOR_DISABLE)
            {
                motor->transmit_data.velocity_des = 0.0f;
                pid_ref = 0.0f;
            }
            else
            {
                motor->transmit_data.velocity_des = pid_ref;
            }

            DM_Speed_Ctrl(motor, motor->transmit_data.velocity_des);
        }

        else if (motor->dm_mode == POS_MODE)
        {
            // 若该电机处于停止状态,直接将buff置零
            if (motor->motor_state_flag == MOTOR_DISABLE)
            {
                motor->transmit_data.position_des = motor->receive_data.position;
                motor->transmit_data.velocity_des = 0.0f;
                pid_ref = 0.0f;
            }
            else
            {
                motor->transmit_data.velocity_des = pid_ref;
            }

            DM_Pos_Speed_Ctrl(motor,
                              motor->transmit_data.position_des,
                              motor->transmit_data.velocity_des);
        }

        else if (motor->dm_mode == MIT_MODE)
        {
            // 若该电机处于停止状态,直接将buff置零
            if (motor->motor_state_flag == MOTOR_DISABLE)
            {
                PID_Init(motor_controller->angle_PID);
                PID_Init(motor_controller->speed_PID);
                motor->transmit_data.position_des = motor->receive_data.position;
                motor->transmit_data.velocity_des = 0.0f;
                motor->transmit_data.torque_des = 0.0f;
                pid_ref = 0.0f;
            }

            // 这里不提供其他的设计，只有纯力矩控制，如有需求在此修改代码逻辑即可
            motor->transmit_data.position_des = 0.0f;
            motor->transmit_data.velocity_des = 0.0f;
            motor->transmit_data.Kp = 0;
            motor->transmit_data.Kd = 0;

            if (motor->motor_settings.outer_loop_type == ANGLE_LOOP)
            {
                motor->transmit_data.torque_des = pid_ref;

                DM_MIT_Ctrl(motor,
                            motor->transmit_data.position_des,
                            motor->transmit_data.velocity_des,
                            motor->transmit_data.Kp,
                            motor->transmit_data.Kp,
                            motor->transmit_data.torque_des);
            }
            if (motor->motor_settings.outer_loop_type == SPEED_LOOP)
            {
                motor->transmit_data.torque_des = pid_ref;

                DM_MIT_Ctrl(motor,
                            motor->transmit_data.position_des,
                            motor->transmit_data.velocity_des,
                            motor->transmit_data.Kp,
                            motor->transmit_data.Kp,
                            motor->transmit_data.torque_des);
            }
            if (motor->motor_settings.outer_loop_type == TORQUE_LOOP)
            {
                motor->transmit_data.torque_des = pid_ref;

                DM_MIT_Ctrl(motor,
                            motor->transmit_data.position_des,
                            motor->transmit_data.velocity_des,
                            motor->transmit_data.Kp,
                            motor->transmit_data.Kp,
                            motor->transmit_data.torque_des);
            }
        }
        else if (motor->dm_mode == DM124_MODE)
        {
            // 若该电机处于停止状态,直接将buff置零
            if (motor->motor_state_flag == MOTOR_DISABLE)
            {
                pid_ref = 0.0f;
            }
            else
            {
                motor->transmit_data.velocity_des = pid_ref;
            }

            DM_DM124_Ctrl(motor,
                          motor->transmit_data.velocity_des);
        }
    }
}
