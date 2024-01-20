/**
 ******************************************************************************
 * @file    hardware_config.cpp/h
 * @brief   Hardware configuration. 硬件配置（端口和硬件相关宏定义）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// 开发板型号
#define DBA

// 遥控器串口
#define RC_UART &huart1
// 视觉通信串口
#define CV_UART &huart3
// 裁判系统通信串口
#define REFEREE_UART &huart6
// 控制器串口
#define CONTROLLER_UART &huart8
//舵机串口
#define SERVO_UART &huart7
// 调试串口
// #define DEBUG_UART &huart8
#define DEBUG_USB
//#define UI_TEST_ON_COMPUTER
// imu相关(spi/i2c/tim)
#define MPU6500_SPI &hspi5
#define BOARD_IMU_HEAT_TIM &htim3
#define BOARD_IMU_HEAT_CHANNEL TIM_CHANNEL_2

#ifdef __cplusplus
}
#endif

#endif  // HARDWARE_CONFIG_H