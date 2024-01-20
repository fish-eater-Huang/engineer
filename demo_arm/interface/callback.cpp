/**
 ******************************************************************************
 * @file    callback.cpp/h
 * @brief   Interrupt request callback. 中断回调管理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "interface/callback.h"

#include "can.h"
#include "gpio.h"
#include "tim.h"

#include "hardware_config.h"

#include "app/arm_controller.h"
#include "app/imu_comm.h"
#include "app/motor_monitor.h"
#include "app/serial_tool.h"
#include "base/cap_comm/cap_comm.h"
#include "base/cv_comm/cv_comm.h"
#include "base/referee_comm/referee_comm.h"
#include "base/remote/remote.h"

extern RC rc;
extern CVComm cv_comm;
extern RefereeComm referee;
extern ImuComm imu_comm;
extern ControllerComm controller_comm;
extern SerialStudio serial_tool;
uint16_t recieve = 0;
// CAN receive callback
// CAN接收回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

  motorsCanRxMsgHandle(hcan, rx_header, rx_data);
  if (imu_comm.canRxMsgCheck(hcan, rx_header)) {
    imu_comm.canRxMsgCallback(hcan, rx_header, rx_data);
  }
}

// UART transmit callback
// UART发送中断回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {}

// UART receive callback
// UART接收中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
	recieve++;
  if (rc.uartCheck(huart)) {
    rc.rxCallback();
  }
  if (cv_comm.uartCheck(huart)) {
    cv_comm.rxCallback();
  }
  if (referee.uartCheck(huart)) {
    referee.rxCallback();
  }
  if (controller_comm.uartCheck(huart)) {
    controller_comm.rxCallback();
  }
  if (serial_tool.uartCheck(huart)) {
    serial_tool.uartRxCallback();
  }
}

// UART idle callback. Called in stm32f4xx_it.c USARTx_IRQHandler()
// UART空闲中断处理，在stm32f4xx_it.c的USARTx_IRQHandler()函数中调用
void User_UART_IdleHandler(UART_HandleTypeDef* huart) {
  // Judge if idle enabled. 判断空闲中断是否使能
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
    // Clear idle flag. 清除空闲中断标记
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    // idle中断回调
    if (rc.uartCheck(huart)) {
      rc.idleCallback();
    }
  }
}

// USB CDC. Called in usbd_cdc_if CDC_Receive_FS()
// USB接收回调函数，在usbd_cdc_if的CDC_Receive_FS()函数中调用
void User_USB_CDC_RxMsgCallback(uint8_t* Buf, uint32_t* Len) {
  serial_tool.usbRxCallback(Buf, *Len);
}

// EXTI callback
// 外部中断回调
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {}
