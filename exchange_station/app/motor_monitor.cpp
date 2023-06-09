/**
 ******************************************************************************
 * @file    motor_monitor.cpp/h
 * @brief   Motor parameter, id config and management. 电机参数id配置和统一管理
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/motor_monitor.h"
#include "lib/arm_math/arm_math.h"

// Motor parameter config
// 电机参数配置

// Chassis motor 底盘电机
const PID chassis_wheel_spid(40, 1, 10, 1000, 16384);
Motor CMFL(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
           PID(), PID(chassis_wheel_spid));             // ppid, spid
Motor CMFR(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
           PID(), PID(chassis_wheel_spid));             // ppid, spid
Motor CMBL(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
           PID(), PID(chassis_wheel_spid));             // ppid, spid
Motor CMBR(Motor::M3508, 3591.f / 187.f, Motor::SPEED,  // type, ratio, method
           PID(), PID(chassis_wheel_spid));             // ppid, spid

// Gimbal motor 云台电机
Motor GMY(Motor::GM6020, 1, Motor::POSITION_SPEED,   // type, ratio, method
          PID(25, 0.1, 10, 10, 1800),                // ppid
          PID(400, 2, 0, 500, 30000),                // spid
          true);                                     // use kf
Motor GMP(Motor::GM6020, -1, Motor::POSITION_SPEED,  // type, ratio, method
          PID(25, 0.1, 10, 10, 1800),                // ppid
          PID(400, 2, 0, 500, 30000),                // spid
          true);                                     // use kf

// Friction wheel motors
const PID fric_spid(6, 0.05, 2, 200, 16384);
Motor FRICL(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
            PID(), PID(fric_spid));         // ppid, spid
Motor FRICR(Motor::M3508, 1, Motor::SPEED,  // type, ratio, method
            PID(), PID(fric_spid));         // ppid, spid

// Stir motor
Motor STIR(Motor::M2006, 36, Motor::POSITION_SPEED,  // type, ratio, method
           PID(20, 0.1, 10, 10, 2500),               // ppid
           PID(60, 0.1, 200, 1000, 10000));          // spid

// Other motors
Motor m1(Motor::M3508, 1, Motor::POSITION_SPEED,  // type, ratio, method
         PID(50, 0.02, 50, 200, 36000),           // ppid
         PID(20, 0.02, 50, 200, 16384));          // spid

// DJI Motor id config, M3508/M2006: 1~8, GM6020: 5~11
// DJI电机ID配置，M3508，M2006可配置范围为1~8，GM6020可配置范围为5~11
Motor* can1_dji_motor[11] = {
    &FRICL,   // id:1
    &FRICR,   // id:2
    nullptr,  // id:3
    nullptr,  // id:4
    &STIR,    // id:5
    &GMP,     // id:6
    nullptr,  // id:7
    &m1,      // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};
Motor* can2_dji_motor[11] = {
    &CMFL,    // id:1
    &CMFR,    // id:2
    &CMBL,    // id:3
    &CMBR,    // id:4
    &GMY,     // id:5
    nullptr,  // id:6
    nullptr,  // id:7
    nullptr,  // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};

DJIMotorDriver dji_motor_driver(can1_dji_motor, can2_dji_motor);

// 电机停转速度上限(dps)
const float motor_stop_rotate_speed_thres = 1200;

// Stop and shut off all motors
// 所有电机先停转再断电
void allMotorsStopShutoff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can1_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can1_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can1_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      if (fabs(can2_dji_motor[i]->motor_data_.rotate_speed) >
          motor_stop_rotate_speed_thres) {
        can2_dji_motor[i]->mode_ = Motor::STOP;
      } else {
        can2_dji_motor[i]->mode_ = Motor::POWEROFF;
      }
    }
  }
}

// Shut off all motors
// 所有电机断电
void allMotorsShutOff(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::POWEROFF;
    }
  }
}

// Stop all motors
// 所有电机停转
void allMotorsStop(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::STOP;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::STOP;
    }
  }
}

// Startup all motors
// 所有电机启动
void allMotorsOn(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr &&
        can1_dji_motor[i]->mode_ != Motor::INIT) {
      can1_dji_motor[i]->mode_ = Motor::WORKING;
    }
    if (can2_dji_motor[i] != nullptr &&
        can2_dji_motor[i]->mode_ != Motor::INIT) {
      can2_dji_motor[i]->mode_ = Motor::WORKING;
    }
  }
}

// Reset all motors.
// 电机统一初始化
void allMotorsInit(void) {
  dji_motor_driver.idConfig();
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->reset();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->reset();
    }
  }
}

// Handle all motors. Called in motorTask
// 电机统一处理，在motorTask中调用
void allMotorsHandle(void) {
  for (int i = 0; i < 11; i++) {
    if (can1_dji_motor[i] != nullptr) {
      can1_dji_motor[i]->handle();
    }
    if (can2_dji_motor[i] != nullptr) {
      can2_dji_motor[i]->handle();
    }
  }
}

// Check CAN channel and id of received CAN message
// 校验接收信息的CAN通道和ID，调用对应回调函数
void motorscanRxMsgHandle(CAN_HandleTypeDef* hcan,
                          CAN_RxHeaderTypeDef rx_header, uint8_t* rx_data) {
  if (dji_motor_driver.canRxMsgCheck(hcan, rx_header)) {
    dji_motor_driver.canRxMsgCallback(hcan, rx_header, rx_data);
  }
}
