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

// 机械臂关节电机
float j1Model(const float& torque, const float& speed = 0) {
  return torque * 4e2f;  // 8192*19.2/(3*100) ~= 5.2e2
}
Motor JM1(Motor::M3508, 100, Motor::POSITION_SPEED,     // type, ratio, method
          PID(20, 0.1, 5, 10, 120),                     // ppid
          PID(200, 0.1, 20, 1000, 16384),               // spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50),  // kf
          j1Model);                                     // model

float j2Model(const float& torque, const float& speed = 0) {
  return torque * 4e2f;  // 8192*19.2/(3*100) ~= 5.2e2
}
Motor JM2(Motor::M3508, -100, Motor::POSITION_SPEED,    // type, ratio, method
          PID(20, 0.1, 5, 10, 90),                      // ppid
          PID(200, 0.1, 20, 1000, 16384),               // spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50),  // kf
          j2Model);                                     // model

float j3Model(const float& torque, const float& speed = 0) {
  return torque * 4e2f;  // 8192*19.2/(3*100) ~= 5.2e2
}
Motor JM3(Motor::M3508, 100, Motor::POSITION_SPEED,     // type, ratio, method
          PID(20, 0.1, 5, 10, 90),                      // ppid
          PID(200, 0.1, 20, 1000, 16384),               // spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50),  // kf
          j3Model);                                     // model

float j4Model(const float& torque, const float& speed = 0) {
  return torque * 2.73e3f;  // 8192*19.2/(3*19.2) ~= 2.73e3
}
Motor JM4(Motor::M3508, 19.2032f, Motor::POSITION_SPEED,  // type, ratio, method
          PID(20, 0.05, 5, 10, 180),                      // ppid
          PID(50, 0.05, 10, 1000, 16384),                 // spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50),    // kf
          j4Model);                                       // model

float j5Model(const float& torque, const float& speed = 0) {
  return torque * 5.24e4f;  // 8192*19.2/(3*1) ~= 5.24e4
}
Motor JM5(Motor::M3508, 1, Motor::POSITION_SPEED,       // type, ratio, method
          PID(30, 0.05, 10, 50, 360),                   // ppid
          PID(30, 0.05, 200, 1000, 16384),              // spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50),  // kf
          j5Model);                                     // model

float j6Model(const float& torque, const float& speed = 0) {
  return torque * 5.24e4f;  // 8192*19.2/(3*1) ~= 5.24e4
}
Motor JM6(Motor::M3508, -1, Motor::POSITION_SPEED,      // type, ratio, method
          PID(20, 0.05, 10, 10, 360),                   // ppid
          PID(5, 0.02, 20, 1000, 16384),                // spid
          true, Motor::KFParam_t(2, 1e4, 1, 0.75, 50),  // kf
          j6Model);                                     // model

Motor M1(Motor::MIT, 1, Motor::TORQUE,  // type, ratio, method
         PID(), PID(), true);           // ppid, spid, use kf

Motor M2(Motor::MIT, 1, Motor::TORQUE,  // type, ratio, method
         PID(), PID(), true);           // ppid, spid, use kf

// DJI Motor id config, M3508/M2006: 1~8, GM6020: 5~11
// DJI电机ID配置，M3508，M2006可配置范围为1~8，GM6020可配置范围为5~11
Motor* can1_dji_motor[11] = {
    &JM1,     // id:1
    &JM2,     // id:2
    &JM3,     // id:3
    &JM4,     // id:4
    &JM5,     // id:5
    &JM6,     // id:6
    nullptr,  // id:7
    nullptr,  // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};
Motor* can2_dji_motor[11] = {
    nullptr,  // id:1
    nullptr,  // id:2
    nullptr,  // id:3
    nullptr,  // id:4
    nullptr,  // id:5
    nullptr,  // id:6
    nullptr,  // id:7
    nullptr,  // id:8
    nullptr,  // id:9
    nullptr,  // id:10
    nullptr   // id:11
};

DJIMotorDriver dji_motor_driver(can1_dji_motor, can2_dji_motor);

// HT04电机参数
namespace ht04 {
// position limit(rad)
const float p_min = -95.5f;
const float p_max = 95.5f;
// velocity limit(rad/s)
const float v_min = -45;
const float v_max = 45;
// PD param limit
const float kp_min = 0;
const float kp_max = 500;
const float kv_min = 0;
const float kv_max = 5;
// feedforward torque limit(N·m)
const float t_ff_min = -40;
const float t_ff_max = 40;
// feedback torque limit(N·m)
const float t_min = -40;
const float t_max = 40;
}  // namespace ht04

// 8112电机参数
namespace m8112 {
// position limit(rad)
const float p_min = -12.5f;
const float p_max = 12.5f;
// velocity limit(rad/s)
const float v_min = -30;
const float v_max = 30;
// PD param limit
const float kp_min = 0;
const float kp_max = 500;
const float kv_min = 0;
const float kv_max = 5;
// feedforward torque limit(N·m)
const float t_ff_min = -10;
const float t_ff_max = 10;
// feedback torque limit(N·m)
const float t_min = -10;
const float t_max = 10;
}  // namespace m8112

// 4310电机参数
namespace m4310 {
// position limit(rad)
const float p_min = -12.5f;
const float p_max = 12.5f;
// velocity limit(rad/s)
const float v_min = -30;
const float v_max = 30;
// PD param limit
const float kp_min = 0;
const float kp_max = 500;
const float kv_min = 0;
const float kv_max = 5;
// feedforward torque limit(N·m)
const float t_ff_min = -7;
const float t_ff_max = 7;
// feedback torque limit(N·m)
const float t_min = -7;
const float t_max = 7;
}  // namespace m4310

// MIT协议电机
MITMotorDriver mit_motor_driver[] = {
    MITMotorDriver(&M1, &hcan1, 0x11, 0x01, m8112::p_min, m8112::p_max,
                   m8112::v_min, m8112::v_max, m8112::kp_min, m8112::kp_max,
                   m8112::kv_min, m8112::kv_max, m8112::t_ff_min,
                   m8112::t_ff_max, m8112::t_min, m8112::t_max),
    MITMotorDriver(&M2, &hcan1, 0x12, 0x02, m8112::p_min, m8112::p_max,
                   m8112::v_min, m8112::v_max, m8112::kp_min, m8112::kp_max,
                   m8112::kv_min, m8112::kv_max, m8112::t_ff_min,
                   m8112::t_ff_max, m8112::t_min, m8112::t_max)};

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
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->mode_ = Motor::POWEROFF;
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
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->mode_ = Motor::POWEROFF;
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
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->mode_ = Motor::STOP;
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
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->mode_ = Motor::WORKING;
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
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->reset();
    mit_motor_driver[i].setCmd(mitmotor::MOTOR_MODE);
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
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    mit_motor_driver[i].motor_->handle();
    if (!mit_motor_driver[i].motor_->connect_.check()) {
      mit_motor_driver[i].setCmd(mitmotor::MOTOR_MODE);
    }
  }
}

// Check CAN channel and id of received CAN message
// 校验接收信息的CAN通道和ID，调用对应回调函数
void motorsCanRxMsgHandle(CAN_HandleTypeDef* hcan,
                          CAN_RxHeaderTypeDef rx_header, uint8_t* rx_data) {
  if (dji_motor_driver.canRxMsgCheck(hcan, rx_header)) {
    dji_motor_driver.canRxMsgCallback(hcan, rx_header, rx_data);
  }
  for (int i = 0; i < sizeof(mit_motor_driver) / sizeof(MITMotorDriver); i++) {
    if (mit_motor_driver[i].canRxMsgCheck(hcan, rx_header)) {
      mit_motor_driver[i].canRxMsgCallback(hcan, rx_header, rx_data);
    }
  }
}
