/**
******************************************************************************
* @file    DJI_motor.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/
#include "DJI_motor.h"

#include "bsp_dwt.h"

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJI_motor_instance_t *dji_motor_instances[DJI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算

void DJI_Motor_Error_Detection(DJI_motor_instance_t *motor);

/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
 *        该变量将在 DJI_Motor_Control() 中使用,分组在 Motor_Sender_Grouping()中进行
 *
 * @note  因为只用于发送,所以不需要在bsp_can中注册
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff *0x1fe,0x2fe(新固件)
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
static CAN_instance_t sender_assignment[15] = {
    [0] = {.can_handle = &hfdcan1, .tx_header.Identifier = 0x1ff, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [1] = {.can_handle = &hfdcan1, .tx_header.Identifier = 0x200, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [2] = {.can_handle = &hfdcan1, .tx_header.Identifier = 0x2ff, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [3] = {.can_handle = &hfdcan2, .tx_header.Identifier = 0x1ff, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [4] = {.can_handle = &hfdcan2, .tx_header.Identifier = 0x200, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [5] = {.can_handle = &hfdcan2, .tx_header.Identifier = 0x2ff, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [6] = {.can_handle = &hfdcan3, .tx_header.Identifier = 0x1ff, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [7] = {.can_handle = &hfdcan3, .tx_header.Identifier = 0x200, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [8] = {.can_handle = &hfdcan3, .tx_header.Identifier = 0x2ff, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [9] = {.can_handle = &hfdcan1, .tx_header.Identifier = 0x1fe, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [10] = {.can_handle = &hfdcan2, .tx_header.Identifier = 0x1fe, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [11] = {.can_handle = &hfdcan3, .tx_header.Identifier = 0x1fe, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [12] = {.can_handle = &hfdcan1, .tx_header.Identifier = 0x2fe, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [13] = {.can_handle = &hfdcan2, .tx_header.Identifier = 0x2fe, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
    [14] = {.can_handle = &hfdcan3, .tx_header.Identifier = 0x2fe, .tx_header.IdType = FDCAN_STANDARD_ID, .tx_header.DataLength = FDCAN_DLC_BYTES_8, .tx_buff = {0}},
};

/**
 * @brief 9个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 Motor_Sender_Grouping()中进行
 */
static uint8_t sender_enable_flag[15] = {0};

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void Motor_Sender_Grouping(DJI_motor_instance_t *motor, can_init_config_t *config)
{
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hfdcan1 ? 1 : (config->can_handle == &hfdcan2 ? 4 : 7);
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hfdcan1 ? 0 : (config->can_handle == &hfdcan2 ? 3 : 6);
        }

        // 计算接收id并设置分组发送id
        config->rx_id = 0x200 + motor_id + 1;   // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        // 检查是否发生id冲突
        for (size_t i = 0; i < idx; ++i)
        {
            // if (dji_motor_instances[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instances[i]->motor_can_instance->rx_id == config->rx_id)
            // {
            // 	uint8_t can_bus = motor->motor_can_instance->can_handle == &hfdcan1 ? 1 : (motor->motor_can_instance->can_handle == &hfdcan2 ? 2 : 3);
            // 	while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
            // 		;
            // }
        }
        break;

    case GM6020:
        if (motor_id < 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            motor_grouping = config->can_handle == &hfdcan1 ? 0 : (config->can_handle == &hfdcan2 ? 3 : 6);
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = config->can_handle == &hfdcan1 ? 2 : (config->can_handle == &hfdcan2 ? 5 : 8);
        }

        // if (motor_id < 4)
        // {
        //     motor_send_num = motor_id;
        //     motor_grouping = config->can_handle == &hfdcan1 ? 9 : (config->can_handle == &hfdcan2 ? 10 : 11);
        // }
        // else
        // {
        //     motor_send_num = motor_id - 4;
        //     motor_grouping = config->can_handle == &hfdcan1 ? 12 : (config->can_handle == &hfdcan2 ? 13 : 14);
        // }

        config->rx_id = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        for (size_t i = 0; i < idx; ++i)
        {
            // if (dji_motor_instances[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instances[i]->motor_can_instance->rx_id == config->rx_id)
            // {
            // 	uint16_t can_bus;
            // 	can_bus = motor->motor_can_instance->can_handle == &hfdcan1 ? 1 : (motor->motor_can_instance->can_handle == &hfdcan2 ? 2 : 3);
            // 	while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
            // 		;
            // }
        }
        break;

    default: // other motors should not be registered here
        while (1)
            ;
    }
}

/**
 * @todo  是否可以简化多圈角度的计算？
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void Decode_DJI_Motor(CAN_instance_t *_instance)
{
    // 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
    // _instance指针指向的id是对应电机instance的地址,通过强制转换为电机instance的指针,再通过->运算符访问电机的成员motor_measure,最后取地址获得指针
    uint8_t *rxbuff = _instance->rx_buff;
    DJI_motor_instance_t *motor = (DJI_motor_instance_t *)_instance->id;
    DJI_motor_callback_t *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

    Supervisor_Reload(motor->supervisor);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->ecd = ((measure->ecd >= measure->offset_ecd) ? (measure->ecd - measure->offset_ecd) : (measure->ecd + 8191 - measure->offset_ecd));

    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    int16_t err = measure->ecd - measure->last_ecd;

    if (err > 4096)
    {
        measure->total_round--;
        measure->total_ecd += -8192 + err;
    }
    else if (err < -4096)
    {
        measure->total_round++;
        measure->total_ecd += 8192 + err;
    }
    else
    {
        measure->total_ecd += err;
    }
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
    measure->total_rad = (measure->total_ecd / 8192.0f) * 2 * PI;
    measure->speed = (measure->speed_aps / 180.0f * PI);
    measure->angle = measure->ecd / 8192.0f * 360.0f; // 角度0~360°
    measure->rad = measure->ecd / 8192.0f * 2 * PI;

    DJI_Motor_Error_Detection(motor);
}

static void DJI_Motor_Lost_Callback(void *motor_ptr)
{
    uint16_t can_bus;
    DJI_motor_instance_t *motor = (DJI_motor_instance_t *)motor_ptr;
    can_bus = motor->motor_can_instance->can_handle == &hfdcan1 ? 1 : (motor->motor_can_instance->can_handle == &hfdcan2 ? 2 : 3);
}

// 电机初始化,返回一个电机实例
DJI_motor_instance_t *DJI_Motor_Init(motor_init_config_t *config)
{
    DJI_motor_instance_t *instance = (DJI_motor_instance_t *)malloc(sizeof(DJI_motor_instance_t));
    memset(instance, 0, sizeof(DJI_motor_instance_t));

    // motor basic setting 电机基本设置
    instance->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
    instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等
    instance->motor_feedback = RAD;

    // motor controller init 电机控制器初始化
    instance->motor_controller.current_PID = PID_Init(config->controller_param_init_config.current_PID);
    instance->motor_controller.speed_PID = PID_Init(config->controller_param_init_config.speed_PID);
    instance->motor_controller.angle_PID = PID_Init(config->controller_param_init_config.angle_PID);
    instance->motor_controller.torque_PID = PID_Init(config->controller_param_init_config.torque_PID);

    // 电机控制闭环时的非电机本身反馈数据指针
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    // 电机控制闭环时的前馈控制器或前馈控制量指针
    instance->motor_controller.torque_feedforward_ptr = config->controller_param_init_config.torque_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

    // 后续增加电机前馈控制器(速度和电流)

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    Motor_Sender_Grouping(instance, &config->can_init_config);

    // 注册电机到CAN总线
    config->can_init_config.can_module_callback = Decode_DJI_Motor; // set callback
    config->can_init_config.id = instance;                          // set id,eq to address(it is IdTypentity)
    instance->motor_can_instance = CAN_Register(&config->can_init_config);

    // 注册守护线程
    supervisor_init_config_t supervisor_config = {
        .handler_callback = DJI_Motor_Lost_Callback,
        .owner_id = instance,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    instance->supervisor = Supervisor_Register(&supervisor_config);

    DJI_Motor_Stop(instance);
    dji_motor_instances[idx++] = instance;
    return instance;
}

/* 电流只能通过电机自带传感器监测,后续考虑加入力矩传感器应变片等 */
void DJI_Motor_Change_Feedback(DJI_motor_instance_t *motor, closeloop_type_e loop, feedback_type_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        ;
}

void DJI_Motor_Stop(DJI_motor_instance_t *motor)
{
    motor->motor_state_flag = MOTOR_DISABLE;
}

void DJI_Motor_Enable(DJI_motor_instance_t *motor)
{
    motor->motor_state_flag = MOTOR_ENABLE;
}

/* 修改电机的实际闭环对象 */
void DJI_Motor_Change_Outerloop(DJI_motor_instance_t *motor, closeloop_type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

// 设置参考值
void DJI_Motor_Set_Ref(DJI_motor_instance_t *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

/*
 * @brief  	获取电机当前值
 * @param	电机结构体地址
 * @param   	style=ORIGIN, 读取原始数据；style=RAD, 弧度制 ,style=DEGREE, 角度制
 * @retval 	电机当前值/是否过度封装/
 */

float DJI_Motor_GetVal(DJI_motor_instance_t *motor,
                       motor_measure_e feedback,
                       DJI_motor_feedback_data_e type)
{
    float val;
    // 角度转换
    if (feedback == MOTOR_ANGLE)
    {
        // 原始数据
        if (type == ORIGIN)
        {
            val = motor->measure.ecd; // 机械角度0~8191
        }
        // 弧度制数据
        if (type == RAD)
        {
            val = (((float)motor->measure.ecd / 8192.0f) * 2 * PI); // 弧度0~2PI
        }
        if (type == DEGREE)
        {
            val = motor->measure.ecd / 8192.0f * 360.0f; // 角度0~360°
        }
    }
    // 速度转换
    else if (feedback == MOTOR_SPEED)
    {
        // 原始数据
        if (type == ORIGIN)
        {
            val = motor->measure.speed; // RPM
        }
        // 弧度制数据
        if (type == RAD)
        {
            val = motor->measure.speed * 2 * PI / 60.0f; // RPM->RAD/S
        }
        if (type == DEGREE)
        {
            val = motor->measure.speed * 360.0f / 60.0f; // RPM-> **°/s
        }
    }
    return val;
}

/*
 * @brief  	设置电机目标值
 *  		被application层的应用调用,给电机设定参考值.
 *        	对于应用,可以将电机视为传递函数为1的设备,不需要关心底层的闭环
 * @param	电机结构体指针
 * @param	目标值
 * @param   ABS->absolute target;
 *          INCR->add from perious target
 * @retval 	无
 */
// 设置参考值
void DJI_Motor_SetTar(DJI_motor_instance_t *motor, float val, motor_reference_e type)
{
    if (type == ABS)
    {
        motor->motor_controller.pid_ref = val;
    }
    else if (type == INCR)
    {
        motor->motor_controller.pid_ref += val;
    }
}

// 异常检测
void DJI_Motor_Error_Detection(DJI_motor_instance_t *motor)
{
    ;
}

// 为所有电机实例计算三环PID,发送控制报文
void DJI_Motor_Control(void)
{
    // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    uint8_t group, num; // 电机组号和组内编号
    DJI_motor_instance_t *motor;
    motor_control_setting_t *motor_setting; // 电机控制参数
    motor_controller_t *motor_controller;   // 电机控制器
    DJI_motor_callback_t *receive_data;     // 电机测量值
    float pid_fab, pid_ref;                 // 电机PID测量值和设定值

    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i)
    { // 减小访存开销,先保存指针引用
        motor = dji_motor_instances[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        receive_data = &motor->measure;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        // 多环目标值是上环输出为下环输入

        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        {
            pid_ref *= -1; // 目标值设置反转
        }
        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
            {
                pid_fab = *motor_controller->other_angle_feedback_ptr;
            }
            else
            {
                if (motor->motor_feedback == ORIGIN)
                {
                    pid_fab = receive_data->total_ecd;
                }
                else if (motor->motor_feedback == RAD)
                {
                    pid_fab = receive_data->rad;
                }
                else if (motor->motor_feedback == DEGREE)
                {
                    // pid_fab = receive_data->total_angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
                    pid_fab = receive_data->angle;
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
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
            {
                pid_ref += *motor_controller->speed_feedforward_ptr;
            }
            if (motor_setting->speed_feedback_source == OTHER_FEED)
            {
                pid_fab = *motor_controller->other_speed_feedback_ptr;
            }
            else
            {
                if (motor->motor_feedback == ORIGIN)
                {
                    pid_fab = receive_data->speed;
                }
                else if (motor->motor_feedback == RAD)
                {
                    pid_fab = receive_data->speed;
                }
                else if (motor->motor_feedback == DEGREE)
                {
                    pid_fab = receive_data->speed_aps;
                }
            }
            // 更新pid_ref进入下一个环
            pid_ref = PID_Increment(motor_controller->speed_PID,
                                    pid_fab,
                                    pid_ref);
        }

        // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
        if (motor_setting->feedforward_flag & TORQUE_FEEDFORWARD)
        {
            pid_ref += *motor_controller->torque_feedforward_ptr;
        }
        if (motor_setting->close_loop_type & TORQUE_LOOP)
        {
            // 采用何种pid需要自己抉择
            pid_ref = PID_Position(motor_controller->torque_PID,
                                   receive_data->real_current,
                                   pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
        {
            pid_ref *= -1; // 输出值设置反转
        }

        /* ------------------------------handler------------------------------------*/
        // 获取最终输出
        motor->target.current = (int16_t)pid_ref;

        // 分组填入发送数据
        group = motor->sender_group;
        num = motor->message_num;
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)(motor->target.current >> 8);         // 低八位
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(motor->target.current & 0x00ff); // 高八位

        // 若该电机处于停止状态,直接将buff置零
        if (motor->motor_state_flag == MOTOR_DISABLE)
        {
            PID_Init(motor_controller->angle_PID);
            PID_Init(motor_controller->speed_PID);
            memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
        }
    }
    /* ------------------------------handler------------------------------------*/

    // 遍历flag,检查是否要发送这一帧报文TODO(GUATAI):后续应解耦，能够由开发者来选择何时发送，来达到每个模块不同控制频率的需求
    for (size_t i = 0; i < 15; ++i)
    {
        if (sender_enable_flag[i])
        {
            CAN_Transmit(&sender_assignment[i], 2);
        }
    }
}
