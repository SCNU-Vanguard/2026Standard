/**
******************************************************************************
 * @file    remote_vt03.c
 * @brief
 * @author
 ******************************************************************************
 */
#include <stdlib.h>
#include <string.h>
#include "remote_vt03.h"

#include "defense_center.h"

#define REMOTE_VT03_FRAME_SIZE 21u //图传遥控器接收的buffer大小, 21字节(168/8),频率1/14ms，921600bps

// 遥控器数据
static VT03_ctrl_t vt03_ctrl[2];     //[0]:当前数据VT03_TEMP,[1]:上一次的数据VT03_LAST.用于按键持续按下和切换的判断

static supervisor_t *vt03_supervisor_instance; // 监视器实例
static USART_t *vt03_usart_instance;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void Rectify_VT03_joystick(void)
{
	for (uint8_t i = 0 ; i < 5 ; ++i)
	{
		if (abs(*(&vt03_ctrl[VT03_TEMP].rc.rocker_l_ + i)) > 660)
		{
			*(&vt03_ctrl[VT03_TEMP].rc.rocker_l_ + i) = 0;
		}
	}
}

/**
 * @brief vt03图传遥控器数据解析
 * @param vt03_buf 接收buffer
 */
static void Vt03_Control_Resolve(const uint8_t *vt03_buf)
{
	//小端  [3]+[2]
	vt03_ctrl[VT03_TEMP].rc.rocker_r_ =( (vt03_buf[2] | (vt03_buf[3] << 8)) & 0x07FF )- VT03_CH_VALUE_OFFSET; //
	vt03_ctrl[VT03_TEMP].rc.rocker_r1 =( ( (vt03_buf[3] >> 3) | (vt03_buf[4] << 5)) & 0x07FF )- VT03_CH_VALUE_OFFSET; //
	vt03_ctrl[VT03_TEMP].rc.rocker_l_ =( ( (vt03_buf[4] >> 6) | (vt03_buf[5] << 2) | (vt03_buf[6] << 10) ) & 0x07FF )- VT03_CH_VALUE_OFFSET; //
	vt03_ctrl[VT03_TEMP].rc.rocker_l1 =( ( (vt03_buf[6] >> 1) | (vt03_buf[7] << 7))  & 0x07FF )- VT03_CH_VALUE_OFFSET; //
	vt03_ctrl[VT03_TEMP].rc.dial      =( ( (vt03_buf[8] >> 1) | (vt03_buf[9] << 7)) & 0x07FF )- VT03_CH_VALUE_OFFSET; // 左侧拨轮
	Rectify_VT03_joystick();

	//VT03新增的按键
	vt03_ctrl[VT03_TEMP].rc.gear_shift  = (vt03_buf[7] >> 4) & 0x03; // 0:空挡 1:N挡 2:S挡
	vt03_ctrl[VT03_TEMP].rc.pause       = (vt03_buf[7] >> 6) & 0x01; // 暂停键
	vt03_ctrl[VT03_TEMP].rc.auto_key_l  = (vt03_buf[7] >> 7) & 0x01; // 自动按键左
	vt03_ctrl[VT03_TEMP].rc.auto_key_r  = vt03_buf[8]  & 0x01; // 自动按键右
	vt03_ctrl[VT03_TEMP].rc.trigge_key  = (vt03_buf[9] >> 4) & 0x01; // 扳机键

	vt03_ctrl[VT03_TEMP].mouse.x       = (vt03_buf[10] | (vt03_buf[11] << 8)); //!< Mouse X axis
	vt03_ctrl[VT03_TEMP].mouse.y       = (vt03_buf[12] | (vt03_buf[13] << 8)); //!< Mouse Y axis
	vt03_ctrl[VT03_TEMP].mouse.z       = (vt03_buf[14] | (vt03_buf[15] << 8)); //!< Mouse Z axis
	
	vt03_ctrl[VT03_TEMP].mouse.press_l = vt03_buf[16] & 0x03;         		//!< Mouse Left Is Press ?
	vt03_ctrl[VT03_TEMP].mouse.press_r = (vt03_buf[16] >> 2) & 0x03;        //!< Mouse Right Is Press ?
	vt03_ctrl[VT03_TEMP].mouse.press_m = (vt03_buf[16] >> 4) & 0x03;        //!< Mouse Middle Is Press ?

	// 位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
	*(uint16_t *) &vt03_ctrl[VT03_TEMP].key[VT03_KEY_PRESS] = (uint16_t) (vt03_buf[17] | (vt03_buf[18] << 8));


	if (vt03_ctrl[VT03_TEMP].key[VT03_KEY_PRESS].ctrl) // ctrl键按下
		vt03_ctrl[VT03_TEMP].key[VT03_KEY_PRESS_WITH_CTRL] = vt03_ctrl[VT03_TEMP].key[VT03_KEY_PRESS];
	else
		memset(&vt03_ctrl[VT03_TEMP].key[VT03_KEY_PRESS_WITH_CTRL], 0, sizeof(Vt_Key_t));

	if (vt03_ctrl[VT03_TEMP].key[VT03_KEY_PRESS].shift) // shift键按下
		vt03_ctrl[VT03_TEMP].key[VT03_KEY_PRESS_WITH_SHIFT] = vt03_ctrl[VT03_TEMP].key[VT03_KEY_PRESS];
	else
		memset(&vt03_ctrl[VT03_TEMP].key[VT03_KEY_PRESS_WITH_SHIFT], 0, sizeof(Vt_Key_t));

	uint16_t key_now             = vt03_ctrl[VT03_TEMP].key[VT03_KEY_PRESS].keys,              // 当前按键是否按下
	         key_VT03_LAST       = vt03_ctrl[VT03_LAST].key[VT03_KEY_PRESS].keys,                 // 上一次按键是否按下
	         key_with_ctrl       = vt03_ctrl[VT03_TEMP].key[VT03_KEY_PRESS_WITH_CTRL].keys, // 当前ctrl组合键是否按下
	         key_with_shift      = vt03_ctrl[VT03_TEMP].key[VT03_KEY_PRESS_WITH_SHIFT].keys, //  当前shift组合键是否按下
	         key_VT03_LAST_with_ctrl  = vt03_ctrl[VT03_LAST].key[VT03_KEY_PRESS_WITH_CTRL].keys, // 上一次ctrl组合键是否按下
	         key_VT03_LAST_with_shift = vt03_ctrl[VT03_LAST].key[VT03_KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

	for (uint16_t i = 0, j = 0x1 ; i < 16 ; j <<= 1, i++)
	{
		if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
			continue;
		// 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
		if ((key_now & j) && !(key_VT03_LAST & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
			vt03_ctrl[VT03_TEMP].key_count[VT03_KEY_PRESS][i]++;
		// 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
		if ((key_with_ctrl & j) && !(key_VT03_LAST_with_ctrl & j))
			vt03_ctrl[VT03_TEMP].key_count[VT03_KEY_PRESS_WITH_CTRL][i]++;
		// 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
		if ((key_with_shift & j) && !(key_VT03_LAST_with_shift & j))
			vt03_ctrl[VT03_TEMP].key_count[VT03_KEY_PRESS_WITH_SHIFT][i]++;
	}

	memcpy(&vt03_ctrl[VT03_LAST], &vt03_ctrl[VT03_TEMP], sizeof(VT03_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

/**
 * @brief 注册到bsp_usart的回调函数中,解包函数处理
 *
 */
static void Vt03_Control_Rx_Callback(void)
{
	Supervisor_Reload(vt03_supervisor_instance);         // 先喂狗

//	/*校验*/
   if(vt03_usart_instance->recv_buff[0] != 0xA9 || vt03_usart_instance->recv_buff[1] != 0x53 
	 	|| Verify_CRC16_Check_Sum(vt03_usart_instance->recv_buff, REMOTE_VT03_FRAME_SIZE) == 0)
	{
			return;
	}
	Vt03_Control_Resolve(vt03_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 */
static void Vt03_RC_Lost_Callback(void *id)
{
	memset(vt03_ctrl, 0, sizeof(vt03_ctrl)); // 清空遥控器数据
	USART_Service_Init(vt03_usart_instance); // 尝试重新启动接收
}

/**
 * @brief 初始化图传遥控器,该函数会将遥控器注册到串口
 * @attention 注意分配正确的串口硬件
 */
VT03_ctrl_t *Vt03_Control_Init(UART_HandleTypeDef *vt03_usart_handle)
{
	usart_init_config_t conf;
	conf.module_callback = Vt03_Control_Rx_Callback;
	conf.usart_handle    = vt03_usart_handle;
	conf.recv_buff_size  = REMOTE_VT03_FRAME_SIZE;
	vt03_usart_instance    = USART_Register(&conf);

	// 进行守护进程的注册,用于定时检查遥控器是否正常工作
	supervisor_init_config_t supervisor_conf = {
		.reload_count = 1000, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
		.handler_callback = Vt03_RC_Lost_Callback,
		.owner_id = &vt03_ctrl, 
	};
	vt03_supervisor_instance = Supervisor_Register(&supervisor_conf);

	return vt03_ctrl;
}

