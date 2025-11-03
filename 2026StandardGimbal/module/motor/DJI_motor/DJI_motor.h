/**
 * @file DJI_motor.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "drv_motor.h"

#include "defense_center.h"

#define DJI_MOTOR_CNT 12

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制

typedef enum
{
    ORIGIN = 0,
    RAD = 1,
    DEGREE = 2,
} DJI_motor_feedback_data_e;

/* DJI电机CAN反馈信息*/
typedef struct
{
    uint16_t last_ecd; // 上一次读取的编码器值
    uint16_t ecd;      // 0-8191,刻度总共有8192格
    uint16_t offset_ecd;
    float angle;   // 角度
    float rad;     // 弧度
    float speed;            // round/s
    float angle_single_round; // 单圈角度
    float speed_aps;          // 角速度,单位为:度/秒
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度 Celsius

    float total_ecd;     // 总编码器值,注意方向
    float total_angle;   // 总角度,注意方向
    float total_rad;     // 总弧度,注意方向
    int32_t total_round; // 总圈数,注意方向
} DJI_motor_callback_t;

/* DJI电机CAN接收信息*/
typedef struct
{
    int16_t current; // 电流
} DJI_motor_fillmessage_t;

/**
 * @brief DJI intelligent motor typedef
 *
 */
typedef struct
{
    motor_type_e motor_type; // 电机类型
    motor_reference_e motor_reference;

    motor_control_setting_t motor_settings; // 电机控制设置
    motor_controller_t motor_controller;    // 电机控制器

    CAN_instance_t *motor_can_instance; // 电机CAN实例

    motor_working_type_e motor_state_flag;              // 启停标志
    motor_error_detection_type_e motor_error_detection; // 异常检测

    DJI_motor_callback_t measure;   // 电机测量值
    DJI_motor_fillmessage_t target; // 电机设定值
    DJI_motor_feedback_data_e motor_feedback;

    supervisor_t *supervisor;

    uint32_t feed_cnt;
    float dt;

    // 分组发送设置
    uint8_t sender_group;
    uint8_t message_num;
} DJI_motor_instance_t;

/**
 * @brief 调用此函数注册一个DJI智能电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *        请注意不要在一条总线上挂载过多的电机(超过6个),若一定要这么做,请降低每个电机的反馈频率(设为500Hz),
 *        并减小DJIMotorControl()任务的运行频率.
 *
 * @attention M3508和M2006的反馈报文都是0x200+id,而GM6020的反馈是0x204+id,请注意前两者和后者的id不要冲突.
 *            如果产生冲突,在初始化电机的时候会进入IDcrash_Handler(),可以通过debug来判断是否出现冲突.
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return DJI_motor_instance_t*
 */
DJI_motor_instance_t *DJI_Motor_Init(motor_init_config_t *config);

/**
 * @brief 被application层的应用调用,给电机设定参考值.
 *        对于应用,可以将电机视为传递函数为1的设备,不需要关心底层的闭环
 *
 * @param motor 要设置的电机
 * @param ref 设定参考值
 */
void DJI_Motor_Set_Ref(DJI_motor_instance_t *motor, float ref);

/**
 * @brief 切换反馈的目标来源,如将角速度和角度的来源换为IMU(小陀螺模式常用)
 *
 * @param motor 要切换反馈数据来源的电机
 * @param loop  要切换反馈数据来源的控制闭环
 * @param type  目标反馈模式
 */
void DJI_Motor_Change_Feedback(DJI_motor_instance_t *motor, closeloop_type_e loop, feedback_type_e type);

/**
 * @brief 该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
void DJI_Motor_Control(void);

/**
 * @brief 停止电机,注意不是将设定值设为零,而是直接给电机发送的电流值置零
 *
 */
void DJI_Motor_Stop(DJI_motor_instance_t *motor);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        初始化时不需要此函数,因为stop_flag的默认值为0
 *
 */
void DJI_Motor_Enable(DJI_motor_instance_t *motor);

/**
 * @brief 修改电机闭环目标(外层闭环)
 *
 * @param motor  要修改的电机实例指针
 * @param outer_loop 外层闭环类型
 */
void DJI_Motor_Change_Outerloop(DJI_motor_instance_t *motor, closeloop_type_e outer_loop);
float DJI_Motor_GetVal(DJI_motor_instance_t *motor,
                       motor_measure_e feedback,
                       DJI_motor_feedback_data_e type);

#endif /* __DJI_MOTOR_H__ */
