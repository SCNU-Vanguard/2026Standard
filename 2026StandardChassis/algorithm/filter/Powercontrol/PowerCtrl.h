#ifndef POWERCTRL_H
#define POWERCTRL_H

// 3508电机功率模型参数

#define K0_3508 1.996889994e-06f  // k0  ---  (20/16384)*(0.3)*(187/3591)/9.55    //转矩常数 = 0.3 ； 9.55 = 360/(2*PI)    
                             //           9.55 * P = n *T;  T = 0.3 *I;  n = n1*187/3591
//#define K1 1.23e-07f  // k1
//#define K2 1.453e-07f  // k2

#define K1_3508 2.0326e-07  // k1
#define K2_3508 1.453e-07f  // k2

#define K0_6020 1.996889994e-06f
#define K1_6020 2.0326e-07  // k1
#define K2_6020 1.453e-07f  // k2


#define C_3508 1.25f  // 常量  底盘静态功率/电机数量(4个)
#define C_6020 1.25f  // 常量  底盘静态功率/电机数量(4个)

void chassis_power_control(void);
void Power_Control(void);
typedef enum
{
	SUPERCAP_CONTROL = 0,
	CODE_CONTROL = 1,
} power_control_mode_e;//功率控制模式

#endif