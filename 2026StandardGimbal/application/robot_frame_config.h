/**
* @file robot_frame_config.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __ROBOT_FRAME_CONFIG_H__
#define __ROBOT_FRAME_CONFIG_H__

#define RAD_2_DEGREE 57.2957795f    //rad/s to °/s 180/pi
#define DEGREE_2_RAD 0.01745329252f //°/s to rad/s pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f       //rpm to °/s ×360°/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f //rpm to rad/s ×2pi/60sec

#define BMI088_Frame 1

#endif /* __ROBOT_FRAME_CONFIG_H__ */