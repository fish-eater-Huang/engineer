/**
 ******************************************************************************
 * @file    robot.cpp/h
 * @brief   Main program. 主程序
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "interface/robot.h"

#include "cmsis_os.h"
#include "hardware_config.h"

#include "app/arm.h"
#include "app/arm_controller.h"
#include "app/can_monitor.h"
#include "app/client_ui.h"
#include "app/control.h"
#include "app/imu_comm.h"
#include "app/imu_monitor.h"
#include "app/motor_monitor.h"
#include "app/serial_tool.h"
#include "base/cap_comm/cap_comm.h"
#include "base/cv_comm/cv_comm.h"
#include "base/referee_comm/referee_comm.h"
#include "base/referee_comm/referee_ui.h"
#include "base/remote/remote.h"
#include "base/servo/servo.h"

#ifdef RC_UART
RC rc(RC_UART);
#else
RC rc;
#endif  // RC_UART
#ifdef CV_UART
CVComm cv_comm(CV_UART);
#else
CVComm cv_comm;
#endif  // CV_UART
#ifdef REFEREE_UART
RefereeComm referee(REFEREE_UART);
UI ui(REFEREE_UART, &referee, ui_func, sizeof(ui_func) / sizeof(void*));
#else
RefereeComm referee;
#endif  // REFEREE_UART
#ifdef CONTROLLER_UART
ControllerComm controller_comm(CONTROLLER_UART);
#else
ControllerComm controller_comm;
#endif  // CONTROLLER_UART
#ifdef SERVO_UART
ServoZX361D pump_servo[3] = {ServoZX361D(SERVO_UART), ServoZX361D(SERVO_UART),
                             ServoZX361D(SERVO_UART)};
#else
ServoZX361D pump_servo[3];
#endif  // SERVO_UART
#ifdef DEBUG_UART
SerialStudio serial_tool(DEBUG_UART);
#else
SerialStudio serial_tool;
#endif  // DEBUG_UART

// imu通信（CAN）
ImuComm imu_comm(&hcan2, &arm_imu[0], &arm_imu[1], &arm_imu[2]);
// 机械臂
Arm arm(&JM1, &JM2, &JM3, &JM4, &JM5, &JM6, &board_imu, &arm_imu[0],
        &arm_imu[1], &imu_comm);
// 机械臂控制器
ArmController arm_controller(&controller_comm, controller_imu);

/* FreeRTOS tasks-----------------------------------------------------------*/
osThreadId controlTaskHandle;
void controlTask(void const* argument) {
  rc.init();
  controlInit();
  for (;;) {
    rc.handle();
    controlLoop();
    osDelay(1);
  }
}

osThreadId motorTaskHandle;
void motorTask(void const* argument) {
  allMotorsInit();
  for (;;) {
    allMotorsHandle();
    osDelay(1);
  }
}

osThreadId canTaskHandle;
void canTask(void const* argument) {
  canFilterInit();
  for (;;) {
    canTxMonitor();
  }
}

osThreadId imuTaskHandle;
void imuTask(void const* argument) {
  imu::initAll();
  for (;;) {
    imu::handleAll();
    osDelay(1);
  }
}

osThreadId armTaskHandle;
void armTask(void const* argument) {
  osDelay(2000);
  arm.init();
  for (;;) {
    arm.handle();
    osDelay(1);
  }
}

osThreadId minipcCommTaskHandle;
void minipcCommTask(void const* argument) {
  uint32_t tick = osKernelSysTick();
  cv_comm.init();
  for (;;) {
    cv_comm.txMonitor(&tick);
  }
}

osThreadId refereeCommTaskHandle;
void refereeCommTask(void const* argument) {
  referee.init();
//  ui.init();
  for (;;) {
    referee.handle();
//    ui.handle();
    osDelay(1);
  }
}

osThreadId ctrlCommTaskHandle;
void ctrlCommTask(void const* argument) {
  controller_comm.init();
  uint32_t tick = osKernelSysTick();
  for (;;) {
    controller_comm.handle();
    osDelayUntil(&tick, 40);
  }
}

osThreadId serialToolTaskHandle;
void serialToolTask(void const* argument) {
  uint32_t tick = osKernelSysTick();
  for (;;) {
    serial_tool.handle();
    osDelayUntil(&tick, 40);
  }
}

// Create and config tasks
void rtosTaskInit(void) {
  osThreadDef(control_task, controlTask, osPriorityAboveNormal, 0, 512);
  controlTaskHandle = osThreadCreate(osThread(control_task), NULL);

  osThreadDef(motor_task, motorTask, osPriorityHigh, 0, 128);
  motorTaskHandle = osThreadCreate(osThread(motor_task), NULL);

  osThreadDef(can_task, canTask, osPriorityHigh, 0, 128);
  canTaskHandle = osThreadCreate(osThread(can_task), NULL);

  osThreadDef(imu_task, imuTask, osPriorityRealtime, 0, 512);
  imuTaskHandle = osThreadCreate(osThread(imu_task), NULL);

  osThreadDef(arm_task, armTask, osPriorityNormal, 0, 1800);
  armTaskHandle = osThreadCreate(osThread(arm_task), NULL);

  osThreadDef(minipc_comm_task, minipcCommTask, osPriorityNormal, 0, 512);
  minipcCommTaskHandle = osThreadCreate(osThread(minipc_comm_task), NULL);

  osThreadDef(referee_comm_task, refereeCommTask, osPriorityNormal, 0, 512);
  refereeCommTaskHandle = osThreadCreate(osThread(referee_comm_task), NULL);

  osThreadDef(ctrl_comm_task, ctrlCommTask, osPriorityNormal, 0, 256);
  ctrlCommTaskHandle = osThreadCreate(osThread(ctrl_comm_task), NULL);

  osThreadDef(serial_tool_task, serialToolTask, osPriorityLow, 0, 1024);
  serialToolTaskHandle = osThreadCreate(osThread(serial_tool_task), NULL);
}
