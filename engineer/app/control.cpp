/**
 ******************************************************************************
 * @file    control.cpp/h
 * @brief   Robot control design. 机器人控制设计（模式/键位）
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/control.h"

#include "iwdg.h"

#include "app/chassis.h"
#include "app/gimbal.h"
#include "app/imu_comm.h"
#include "app/imu_monitor.h"
#include "app/mine.h"
#include "app/motor_monitor.h"
#include "base/bsp/bsp_buzzer.h"
#include "base/bsp/bsp_led.h"
#include "base/common/math.h"
#include "base/cv_comm/cv_comm.h"
#include "base/referee_comm/referee_comm.h"
#include "base/remote/remote.h"
#include "base/servo/servo.h"

void iwdgHandler(bool iwdg_refresh_flag);
void robotPowerStateFSM(bool stop_flag);
void robotReset(void);
bool robotStartup(void);
void robotControl(void);

void rcArmControl(uint8_t type);
void controllerArmControl(void);
void trajSetPose(Pose_t pose);
void trajSetJoint(Matrixf<6, 1> q);

void boardLedHandle(void);

extern RC rc;
extern Arm arm;
extern ArmController arm_controller;
extern CVComm cv_comm;
extern RefereeComm referee;
// extern ServoZX361D pump_servo[];

MecanumChassis chassis(&CMFL, &CMFR, &CMBL, &CMBR, PID(10, 0, 10, 100, 270),
                       LowPassFilter(5e-3f));

ArmGimbal gimbal(&JM0, &GMP, &board_imu);

ServoPwm pump_e_servo(&htim5, TIM_CHANNEL_3);
ServoPwm pump_0_servo(&htim5, TIM_CHANNEL_4);
Pump pump_e(&PME, &pump_e_servo, nullptr, 1166, 550, 1166);  // 机械臂气泵
Pump pump_0(&PM0, &pump_0_servo, nullptr, 1166, 1760, 540);  // 存矿气泵

ArmTask task;

BoardLed led;

// 上电状态
enum RobotPowerState_e {
  STOP = 0,
  STARTUP = 1,
  WORKING = 2,
} robot_state;
// 初始化标志
bool startup_flag = false;
// 遥控器挡位记录
RC::RCSwitch last_rc_switch;

// 遥控器控制参数
namespace rcctrl {
float arm_position_rate = 1e-6f;
float arm_direction_rate = 3e-6f;
float arm_joint_rate = 3e-6f;

float chassis_speed_rate = 4.5e-3f;
float chassis_rotate_rate = 0.72f;
float chassis_follow_ff_rate = 0.3f;

float gimbal_rate = 3e-4f;
}  // namespace rcctrl

// 键鼠控制参数
namespace kbctrl {
float chassis_speed = 2.5f;  // m/s

float chassis_fast_ratio = 1.5f;
float chassis_slow_ratio = 0.5f;

float gimbal_rate = 0;
}  // namespace kbctrl

// 机械臂轨迹规划参数
float arm_traj_speed = 0.45f;      // m/s
float arm_traj_rotate_speed = PI;  // rad/s
float arm_traj_q_D1[6] = {math::dps2radps(120), math::dps2radps(90),
                          math::dps2radps(90),  math::dps2radps(270),
                          math::dps2radps(270), math::dps2radps(270)};  // rad/s

// 气泵电机转速参数
float pump_motor_speed = 18000;  // dps

// 控制初始化
void controlInit(void) {
  gimbal.initAll();
  pump_0_servo.init();
  pump_e_servo.init();
  led.init();
  robot_state = STOP;
}

// 控制主循环
void controlLoop(void) {
  iwdgHandler(rc.connect_.check());
  robotPowerStateFSM(!rc.connect_.check() || rc.switch_.r == RC::DOWN);

  if (robot_state == STOP) {
    allMotorsStopShutoff();
    robotReset();
  } else if (robot_state == STARTUP) {
    allMotorsOn();                  // 电机上电
    startup_flag = robotStartup();  // 开机状态判断
  } else if (robot_state == WORKING) {
    allMotorsOn();   // 电机上电
    robotControl();  // 机器人控制
  }

  chassis.rotateHandle(
      -gimbal.j0EncoderAngle() - math::rad2deg(arm.fdb_.q[0][0]),
      -gimbal.j0EncoderAngle());
  chassis.handle();
  JM0.control_data_.feedforward_intensity = JM1.intensity_float_;
  gimbal.handle();
  arm_controller.handle();
  boardLedHandle();

  // 记录遥控器挡位状态
  last_rc_switch = rc.switch_;
}

// IWDG处理，true持续刷新，false进入STOP状态并停止刷新
void iwdgHandler(bool iwdg_refresh_flag) {
  if (!iwdg_refresh_flag) {
    robot_state = STOP;
  } else {
    HAL_IWDG_Refresh(&hiwdg);
  }
}

// STOP(断电，安全模式)/开机/正常运行 状态机
void robotPowerStateFSM(bool stop_flag) {
  if (robot_state == STOP) {
    if (!stop_flag) {
      robot_state = STARTUP;
    }
  } else if (robot_state == STARTUP) {
    if (stop_flag) {
      robot_state = STOP;
    } else if (startup_flag) {
      // 初始化/复位完成
      robot_state = WORKING;
    }
  } else if (robot_state == WORKING) {
    if (stop_flag) {
      robot_state = STOP;
    }
  }
}

// 重置各功能状态
void robotReset(void) {
  startup_flag = false;

  arm.mode_ = Arm::Mode_e::STOP;
  arm.trajAbort();
  gimbal.initJ0();
  pump_e.setMotorSpeed(0);
  pump_e.setValve(Pump::ValveState_e::CLOSE);
  pump_0.setMotorSpeed(0);
  pump_0.setValve(Pump::ValveState_e::CLOSE);
  task.reset();
}

// 开机上电启动处理
bool robotStartup(void) {
  bool flag = true;
  if (!gimbal.init_.j0_finish) {
    chassis.lock_ = true;
    flag = false;
  } else {
    chassis.lock_ = false;
  }
  if (!gimbal.init_.pitch_finish) {
    flag = false;
  }
  if (HAL_GetTick() < 1000) {
    flag = false;
  }
  return flag;
}

// 机器人控制
void robotControl(void) {
  // 遥控器挡位左上右上
  if (rc.switch_.l == RC::UP && rc.switch_.r == RC::UP) {
    // 机械臂x+y+yaw+pitch+roll
    rcArmControl(1);
    if (rc.switch_.l != last_rc_switch.l || rc.switch_.r != last_rc_switch.r) {
      arm.trajAbort();
    }
  }
  // 遥控器挡位左中右上
  else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::UP) {
    // 机械臂x+y+z+yaw+pitch
    rcArmControl(0);
    if (rc.switch_.l != last_rc_switch.l || rc.switch_.r != last_rc_switch.r) {
      arm.trajAbort();
    }
  }
  // 遥控器挡位左下右上
  else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::UP) {
    // 控制器档
    arm.mode_ = Arm::Mode_e::MANIPULATION;
    controllerArmControl();
    if (rc.switch_.l != last_rc_switch.l || rc.switch_.r != last_rc_switch.r) {
      arm.trajAbort();
    }
  }
  // 遥控器挡位左上右中
  else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
    // 云台底盘测试
    arm.mode_ = Arm::Mode_e::JOINT;
    if (rc.switch_.l != last_rc_switch.l || rc.switch_.r != last_rc_switch.r) {
      task.switchMode(ArmTask::Mode_e::MOVE);
    }
    task.handle();

    chassis.lock_ = false;
    chassis.mode_ = MecanumChassis::FOLLOW;
    if (fabs(rc.channel_.dial_wheel) > 100) {
      chassis.mode_ = MecanumChassis::GYRO;
      chassis.setSpeed(rc.channel_.r_col * rcctrl::chassis_speed_rate,
                       -rc.channel_.r_row * rcctrl::chassis_speed_rate,
                       rc.channel_.dial_wheel * rcctrl::chassis_rotate_rate);
    } else {
      chassis.setAngleSpeed(rc.channel_.r_col * rcctrl::chassis_speed_rate,
                            -rc.channel_.r_row * rcctrl::chassis_speed_rate, 0);
    }

    gimbal.addAngle(-rc.channel_.l_row * rcctrl::gimbal_rate,
                    -rc.channel_.l_col * rcctrl::gimbal_rate);
  }
  // 遥控器挡位左中右中
  else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::MID) {
    // arm.mode_ = Arm::Mode_e::JOINT;
    // arm.addJointRef(-rc.channel_.l_row * rcctrl::arm_joint_rate,
    //                 rc.channel_.l_col * rcctrl::arm_joint_rate,
    //                 rc.channel_.dial_wheel * rcctrl::arm_joint_rate,
    //                 rc.channel_.r_row * rcctrl::arm_joint_rate,
    //                 -rc.channel_.r_col * rcctrl::arm_joint_rate, 0);
    // arm.trajAbort();

    // 取矿/三连取矿
    if (rc.channel_.l_col > 500) {
      if (rc.channel_.dial_wheel < -300) {
        task.switchMode(ArmTask::Mode_e::PICK_HIGH);
      } else if (rc.channel_.dial_wheel > 300) {
        task.switchMode(ArmTask::Mode_e::PICK_LOW);
      } else {
        task.switchMode(ArmTask::Mode_e::PICK_NORMAL);
      }
    } else if (rc.channel_.l_col < -500) {
      if (rc.channel_.dial_wheel > 300) {
        task.startPick(ArmTask::PickMethod_e::TRIPLE);
      } else {
        task.startPick(ArmTask::PickMethod_e::SINGLE);
      }
    }

    // 存矿
    if (rc.channel_.l_row < -500) {
      task.switchMode(ArmTask::Mode_e::DEPOSIT_0);
    } else if (rc.channel_.l_row > 500) {
      task.switchMode(ArmTask::Mode_e::DEPOSIT_1);
    }

    // 出矿
    if (rc.channel_.r_row < -500) {
      task.switchMode(ArmTask::Mode_e::WITHDRAW_0);
    } else if (rc.channel_.r_row > 500) {
      task.switchMode(ArmTask::Mode_e::WITHDRAW_1);
    }

    // 兑换
    if (rc.channel_.r_col > 500) {
      task.switchMode(ArmTask::Mode_e::EXCHANGE);
    } else if (rc.channel_.l_col < -500) {
      task.startExchange();
    }

    task.handle();
  }
  // 遥控器挡位左下右中
  else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::MID) {
    // 功能测试
    arm.mode_ = Arm::Mode_e::COMPLIANCE;

    // 存矿气泵
    if (pump_0.valve_state_ == Pump::ValveState_e::CLOSE) {
      if (rc.channel_.r_row < -500) {
        pump_0.setValve(Pump::OPEN_0);
      } else if (rc.channel_.r_row > 500) {
        pump_0.setValve(Pump::OPEN_1);
      }
    } else if (pump_0.valve_state_ == Pump::ValveState_e::OPEN_0) {
      if (rc.channel_.r_row > 300) {
        pump_0.setValve(Pump::CLOSE);
      }
    } else if (pump_0.valve_state_ == Pump::ValveState_e::OPEN_1) {
      if (rc.channel_.r_row < -300) {
        pump_0.setValve(Pump::CLOSE);
      }
    }
    if (rc.channel_.r_col > 500) {
      pump_0.setMotorSpeed(pump_motor_speed);
      task.resetDepositCnt();  // 手动重设存矿出矿计数
    } else if (rc.channel_.r_col < -300) {
      pump_0.setMotorSpeed(0);
    }

    // 机械臂气泵
    if (pump_e.valve_state_ == Pump::ValveState_e::CLOSE) {
      if (rc.channel_.l_row > 500) {
        pump_e.setValve(Pump::OPEN_0);
      }
    } else if (pump_e.valve_state_ == Pump::ValveState_e::OPEN_0) {
      if (rc.channel_.l_row < -300) {
        pump_e.setValve(Pump::CLOSE);
      }
    } else if (pump_e.valve_state_ == Pump::ValveState_e::OPEN_1) {
      pump_e.setValve(Pump::CLOSE);
    }
    if (rc.channel_.l_col > 500) {
      pump_e.setMotorSpeed(pump_motor_speed);
    } else if (rc.channel_.l_col < -300) {
      pump_e.setMotorSpeed(0);
    }
  }
}

// 遥控器机械臂控制，0-全移动自由度，1-全旋转自由度
void rcArmControl(uint8_t type) {
  if (type == 0) {
    if (arm.mode_ == Arm::Mode_e::MANIPULATION) {
      arm.addRef(rc.channel_.l_col * rcctrl::arm_position_rate,
                 -rc.channel_.l_row * rcctrl::arm_position_rate,
                 -rc.channel_.dial_wheel * rcctrl::arm_position_rate,
                 -rc.channel_.r_row * rcctrl::arm_direction_rate,
                 -rc.channel_.r_col * rcctrl::arm_direction_rate, 0);
    } else if (arm.mode_ == Arm::Mode_e::JOINT) {
      arm.addJointRef(-rc.channel_.l_row * rcctrl::arm_joint_rate,
                      rc.channel_.l_col * rcctrl::arm_joint_rate,
                      rc.channel_.dial_wheel * rcctrl::arm_joint_rate,
                      rc.channel_.r_row * rcctrl::arm_joint_rate,
                      -rc.channel_.r_col * rcctrl::arm_joint_rate, 0);
    }
  } else {
    if (arm.mode_ == Arm::Mode_e::MANIPULATION) {
      arm.addRef(rc.channel_.l_col * rcctrl::arm_position_rate,
                 -rc.channel_.l_row * rcctrl::arm_position_rate, 0,
                 -rc.channel_.r_row * rcctrl::arm_direction_rate,
                 -rc.channel_.r_col * rcctrl::arm_direction_rate,
                 -rc.channel_.dial_wheel * rcctrl::arm_direction_rate);
    } else if (arm.mode_ == Arm::Mode_e::JOINT) {
      arm.addJointRef(-rc.channel_.l_row * rcctrl::arm_joint_rate,
                      rc.channel_.l_col * rcctrl::arm_joint_rate,
                      rc.channel_.dial_wheel * rcctrl::arm_joint_rate, 0,
                      -rc.channel_.r_col * rcctrl::arm_joint_rate,
                      rc.channel_.r_row * rcctrl::arm_joint_rate);
    }
  }
}

// 控制器机械臂控制
void controllerArmControl(void) {
  if (arm_controller.state_) {
    if (rc.switch_.l != last_rc_switch.l || rc.switch_.r != last_rc_switch.r) {
      arm_controller.setOffset(arm.fdb_.x - arm_controller.raw_.x,
                               arm.fdb_.y - arm_controller.raw_.y,
                               arm.fdb_.z - arm_controller.raw_.z);
    } else {
      arm_controller.addOffset(
          rc.channel_.l_col * rcctrl::arm_position_rate,
          -rc.channel_.l_row * rcctrl::arm_position_rate,
          -rc.channel_.dial_wheel * rcctrl::arm_position_rate);
    }
    arm.setRef(arm_controller.ref_.x, arm_controller.ref_.y,
               arm_controller.ref_.z, arm_controller.ref_.yaw,
               arm_controller.ref_.pitch, arm_controller.ref_.roll);
  }
}

// 设置机械臂轨迹终点位姿
void trajSetPose(Pose_t pose) {
  arm.trajSet(pose.x, pose.y, pose.z, pose.yaw, pose.pitch, pose.roll,
              arm_traj_speed, arm_traj_rotate_speed);
}

// 设置机械臂轨迹终点关节角度
void trajSetJoint(Matrixf<6, 1> q) {
  arm.trajSet(q, arm_traj_q_D1);
}

// 初始化
void ArmTask::reset(void) {
  // 初始化为移动模式
  mode_ = MOVE;
  // 重置任务状态
  move_.state = IDLE;
  pick_.state = IDLE;
  deposit_.state = IDLE;
  withdraw_.state = IDLE;
  exchange_.state = IDLE;
  triple_pick_.state = IDLE;
  // 重置任务步骤
  move_.step = Move_t::PREPARE;
  pick_.step = Pick_t::PREPARE;
  deposit_.step = Deposit_t::PREPARE;
  withdraw_.step = Withdraw_t::PREPARE;
  exchange_.step = Exchange_t::PREPARE;
  triple_pick_.step = TriplePick_t::PREPARE;
  // 设置指针
  move_.finish_tick = &finish_tick_;
  pick_.finish_tick = &finish_tick_;
  deposit_.finish_tick = &finish_tick_;
  withdraw_.finish_tick = &finish_tick_;
  exchange_.finish_tick = &finish_tick_;
}

// 切换任务模式
void ArmTask::switchMode(ArmTask::Mode_e mode) {
  // 切换至移动模式
  if (mode == MOVE) {
    // 设置任务模式
    mode_ = mode;
    move_.state = WORKING;
    move_.step = Move_t::PREPARE;
    finish_tick_ = HAL_GetTick() + 100;
    // 重置任务状态
    // move_.state = IDLE;
    pick_.state = IDLE;
    deposit_.state = IDLE;
    withdraw_.state = IDLE;
    exchange_.state = IDLE;
    triple_pick_.state = IDLE;
  }
  // 切换至取矿模式
  else if (mode == PICK_NORMAL || mode == PICK_HIGH || mode == PICK_LOW) {
    // 设置任务模式
    mode_ = mode;
    pick_.state = WORKING;
    pick_.step = Pick_t::PREPARE;
    if (mode == PICK_NORMAL) {
      pick_.type = 0;
    } else if (mode == PICK_HIGH) {
      pick_.type = 1;
    } else if (mode == PICK_LOW) {
      pick_.type = 2;
    }
    finish_tick_ = HAL_GetTick() + 100;
    // 重置任务状态
    move_.state = IDLE;
    // pick_.state = IDLE;
    deposit_.state = IDLE;
    withdraw_.state = IDLE;
    exchange_.state = IDLE;
    triple_pick_.state = IDLE;
  }
  // 切换至存矿模式
  else if (mode == DEPOSIT_0 || mode == DEPOSIT_1) {
    // 设置任务模式
    mode_ = mode;
    deposit_.state = WORKING;
    deposit_.step = Deposit_t::PREPARE;
    if (mode == DEPOSIT_0) {
      deposit_.side = 0;
    } else if (mode == DEPOSIT_1) {
      deposit_.side = 1;
    }
    finish_tick_ = HAL_GetTick() + 100;
    // 重置任务状态
    move_.state = IDLE;
    pick_.state = IDLE;
    // deposit_.state = IDLE;
    withdraw_.state = IDLE;
    exchange_.state = IDLE;
    triple_pick_.state = IDLE;
  }
  // 切换至出矿模式
  else if (mode == WITHDRAW_0 || mode == WITHDRAW_1) {
    // 设置任务模式
    mode_ = mode;
    withdraw_.state = WORKING;
    withdraw_.step = Withdraw_t::PREPARE;
    if (mode == WITHDRAW_0) {
      withdraw_.side = 0;
    } else if (mode == WITHDRAW_1) {
      withdraw_.side = 1;
    }
    finish_tick_ = HAL_GetTick() + 100;
    // 重置任务状态
    move_.state = IDLE;
    pick_.state = IDLE;
    deposit_.state = IDLE;
    // withdraw_.state = IDLE;
    exchange_.state = IDLE;
    triple_pick_.state = IDLE;
  }
  // 切换至兑换模式
  else if (mode == EXCHANGE) {
    // 设置任务模式
    mode_ = mode;
    exchange_.state = WORKING;
    exchange_.step = Exchange_t::PREPARE;
    finish_tick_ = HAL_GetTick() + 100;
    // 重置任务状态
    move_.state = IDLE;
    pick_.state = IDLE;
    deposit_.state = IDLE;
    withdraw_.state = IDLE;
    // exchange_.state = IDLE;
    triple_pick_.state = IDLE;
  }
}

// 任务处理
void ArmTask::handle(void) {
  if (mode_ == MOVE) {
    arm.mode_ = Arm::Mode_e::JOINT;
  } else {
    arm.mode_ = Arm::Mode_e::MANIPULATION;
  }
  move_.handle();
  pick_.handle();
  deposit_.handle();
  withdraw_.handle();
  exchange_.handle();
  triplePickHandle();
}

// 开始取矿
void ArmTask::startPick(PickMethod_e method) {
  if (pick_.state == IDLE) {
    return;
  }
  if (pick_.step != Pick_t::LOCATE) {
    return;
  }
  // 设置取矿定位位姿
  pick_.start_pose.x = arm.fdb_.x;
  pick_.start_pose.y = arm.fdb_.y;
  pick_.start_pose.z = arm.fdb_.z;
  pick_.start_pose.yaw = arm.fdb_.yaw;
  pick_.start_pose.pitch = arm.fdb_.pitch;
  pick_.start_pose.roll = arm.fdb_.roll;
  pick_.pick_up_pose = pick_.start_pose;
  pick_.pick_up_pose.z = pick_.start_pose.z + pick_.pick_up_dist;
  // 设置步骤完成时间
  finish_tick_ = HAL_GetTick() + 100;
  if (method == TRIPLE) {
    // 设置三连任务状态
    triple_pick_.state = WORKING;
    triple_pick_.step = TriplePick_t::PICK_1;
    // 设置三连矿石位姿
    triple_pick_.mine[0] = pick_.start_pose;
    triple_pick_.mine[1] = pick_.start_pose;
    triple_pick_.mine[1].x =
        pick_.start_pose.x -
        triple_pick_.mine_dist * sinf(pick_.start_pose.yaw);
    triple_pick_.mine[1].y =
        pick_.start_pose.y +
        triple_pick_.mine_dist * cosf(pick_.start_pose.yaw);
    triple_pick_.mine[2] = pick_.start_pose;
    triple_pick_.mine[2].x =
        pick_.start_pose.x +
        triple_pick_.mine_dist * sinf(pick_.start_pose.yaw);
    triple_pick_.mine[2].y =
        pick_.start_pose.y -
        triple_pick_.mine_dist * cosf(pick_.start_pose.yaw);
  }
}

// 开始兑换
void ArmTask::startExchange(void) {
  if (exchange_.state == IDLE) {
    return;
  }
  // 设置兑换定位位姿
  exchange_.start_pose.x = arm.fdb_.x;
  exchange_.start_pose.y = arm.fdb_.y;
  exchange_.start_pose.z = arm.fdb_.z;
  exchange_.start_pose.yaw = arm.fdb_.yaw;
  exchange_.start_pose.pitch = arm.fdb_.pitch;
  exchange_.start_pose.roll = arm.fdb_.roll;
  exchange_.rotate_pose = exchange_.start_pose;
  exchange_.rotate_pose.pitch =
      exchange_.start_pose.pitch - exchange_.rotate_angle;
  // 设置步骤完成时间
  finish_tick_ = HAL_GetTick() + 100;
}

// 重置存矿计数
void ArmTask::resetDepositCnt(void) {
  deposit_.cnt[0] = withdraw_.cnt[0] + 1;
  deposit_.cnt[1] = withdraw_.cnt[1] + 1;
}

// 存矿气泵电机处理
void ArmTask::depositPumpHandle(void) {
  if (deposit_.cnt[0] > withdraw_.cnt[0] ||
      deposit_.cnt[1] > withdraw_.cnt[1]) {
    pump_0.setMotorSpeed(pump_motor_speed);
  } else {
    pump_0.setMotorSpeed(0);
  }
}

// 移动处理
void ArmTask::Move_t::handle(void) {
  if (state == IDLE) {
    return;
  }

  // 准备阶段
  if (step == PREPARE) {
    if (HAL_GetTick() > *finish_tick) {
      if (fabs(arm.fdb_.q[0][0]) > math::deg2rad(15)) {
        // 机械臂收回轨迹会发生干涉->中间点
        step = TRAJ_RELAY;
        trajSetJoint(q_relay);
      } else {
        // 机械臂收回轨迹不会发生干涉->收回
        step = TRAJ_RETRACT;
        trajSetJoint(q_retract);
      }
      *finish_tick = arm.trajStart();
    }
  }
  // 中间点阶段
  else if (step == TRAJ_RELAY) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_RETRACT;
      trajSetJoint(q_retract);
      *finish_tick = arm.trajStart();
    }
  }
  // 收回阶段
  else if (step == TRAJ_RETRACT) {
    if (HAL_GetTick() > *finish_tick) {
      // arm.trajAbort();
    }
  }
}

// 取矿处理
void ArmTask::Pick_t::handle(void) {
  if (state == IDLE) {
    return;
  }

  // 防止溢出
  if (type > 2) {
    type = 0;
  }

  // 准备阶段
  if (step == PREPARE) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_DEFAULT;
      trajSetPose(default_pose[type]);
      *finish_tick = arm.trajStart() + 500;
    }
  }
  // 移动至默认位置
  else if (step == TRAJ_DEFAULT) {
    if (HAL_GetTick() > *finish_tick) {
      step = LOCATE;
      *finish_tick = UINT32_MAX;  // 等待取矿开始函数
    }
  }
  // 定位
  else if (step == LOCATE) {
    if (HAL_GetTick() > *finish_tick) {
      step = PUMP_E_ON;
      pump_e.setValve(Pump::ValveState_e::CLOSE);
      pump_e.setMotorSpeed(pump_motor_speed);
      *finish_tick = HAL_GetTick() + 1000;
    }
  }
  // 开启机械臂气泵
  else if (step == PUMP_E_ON) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_PICK_UPWARD;
      trajSetPose(pick_up_pose);
      *finish_tick = arm.trajStart();
    }
  }
  // 升高
  else if (step == TRAJ_PICK_UPWARD) {
    if (HAL_GetTick() > *finish_tick) {
      arm.trajAbort();
    }
  }
}

// 存矿处理
void ArmTask::Deposit_t::handle(void) {
  if (state == IDLE) {
    return;
  }

  // 防止溢出
  if (side > 1) {
    side = 0;
  }

  // 准备阶段
  if (step == PREPARE) {
    if (HAL_GetTick() > *finish_tick) {
      if (math::sign(arm.fdb_.y) != math::sign(deposit[side].y) &&
          arm.fdb_.x < 0.1) {
        // 经过中间点0
        step = TRAJ_RELAY_0;
        trajSetPose(relay0);
        *finish_tick = arm.trajStart();
      } else if (arm.fdb_.x > 0.1 || arm.fdb_.z < 0.2) {
        // 经过中间点1
        step = TRAJ_RELAY_1;
        trajSetPose(relay1[side]);
        *finish_tick = arm.trajStart();
      } else {
        // 不经过中间点
        step = TRAJ_DEPOSIT_ABOVE;
        trajSetPose(deposit_above[side]);
        *finish_tick = arm.trajStart();
      }
    }
  }
  // 移动至中间点0
  else if (step == TRAJ_RELAY_0) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_RELAY_1;
      trajSetPose(relay1[side]);
      *finish_tick = arm.trajStart();
    }
  }
  // 移动至中间点1
  else if (step == TRAJ_RELAY_1) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_DEPOSIT_ABOVE;
      trajSetPose(deposit_above[side]);
      *finish_tick = arm.trajStart();
    }
  }
  // 移动至存矿点上方
  else if (step == TRAJ_DEPOSIT_ABOVE) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_DEPOSIT_DOWNWARD;
      trajSetPose(deposit[side]);
      *finish_tick = arm.trajStart();
    }
  }
  // 下降至存矿点
  else if (step == TRAJ_DEPOSIT_DOWNWARD) {
    if (HAL_GetTick() > *finish_tick) {
      step = PUMP_0_ON;
      pump_0.setMotorSpeed(pump_motor_speed);
      pump_0.setValve(Pump::ValveState_e::CLOSE);
      *finish_tick = HAL_GetTick() + 100;
    }
  }
  // 开启存矿气泵
  else if (step == PUMP_0_ON) {
    if (HAL_GetTick() > *finish_tick) {
      step = PUMP_E_OFF;
      pump_0.setMotorSpeed(pump_motor_speed);
      pump_0.setValve(Pump::ValveState_e::CLOSE);
      *finish_tick = HAL_GetTick() + 1000;
    }
  }
  // 关闭机械臂气泵
  else if (step == PUMP_E_OFF) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_DEPOSIT_UPWARD;
      trajSetPose(deposit_above[side]);
      *finish_tick = arm.trajStart();
    }
  }
  // 升高至存矿点上方
  else if (step == TRAJ_DEPOSIT_UPWARD) {
    if (HAL_GetTick() > *finish_tick) {
      arm.trajAbort();
    }
  }
}

// 出矿处理
void ArmTask::Withdraw_t::handle(void) {
  if (state == IDLE) {
    return;
  }

  // 防止溢出
  if (side > 1) {
    side = 0;
  }

  // 准备阶段
  if (step == PREPARE) {
    if (HAL_GetTick() > *finish_tick) {
      if (math::sign(arm.fdb_.y) != math::sign(withdraw[side].y) &&
          arm.fdb_.x < 0.1) {
        // 经过中间点0
        step = TRAJ_RELAY_0;
        trajSetPose(relay0);
        *finish_tick = arm.trajStart();
      } else if (arm.fdb_.x > 0.1 || arm.fdb_.z < 0.2) {
        // 经过中间点1
        step = TRAJ_RELAY_1;
        trajSetPose(relay1[side]);
        *finish_tick = arm.trajStart();
      } else {
        // 不经过中间点
        step = TRAJ_DEPOSIT_ABOVE;
        trajSetPose(withdraw_above[side]);
        *finish_tick = arm.trajStart();
      }
    }
  }
  // 移动至中间点0
  else if (step == TRAJ_RELAY_0) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_RELAY_1;
      trajSetPose(relay1[side]);
      *finish_tick = arm.trajStart();
    }
  }
  // 移动至中间点1
  else if (step == TRAJ_RELAY_1) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_DEPOSIT_ABOVE;
      trajSetPose(withdraw_above[side]);
      *finish_tick = arm.trajStart();
    }
  }
  // 移动至存矿点上方
  else if (step == TRAJ_DEPOSIT_ABOVE) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_DEPOSIT_DOWNWARD;
      trajSetPose(withdraw[side]);
      *finish_tick = arm.trajStart();
    }
  }
  // 下降至存矿点
  else if (step == TRAJ_DEPOSIT_DOWNWARD) {
    if (HAL_GetTick() > *finish_tick) {
      step = PUMP_E_ON;
      pump_e.setMotorSpeed(pump_motor_speed);
      pump_e.setValve(Pump::ValveState_e::CLOSE);
      *finish_tick = HAL_GetTick() + 100;
    }
  }
  // 开启机械臂气泵
  else if (step == PUMP_E_ON) {
    if (HAL_GetTick() > *finish_tick) {
      step = PUMP_0_OFF;
      if (side == 0) {
        pump_0.setValve(Pump::ValveState_e::OPEN_0);
      } else if (side == 1) {
        pump_0.setValve(Pump::ValveState_e::OPEN_1);
      }
      *finish_tick = HAL_GetTick() + 1000;
    }
  }
  // 关闭机械臂气泵
  else if (step == PUMP_0_OFF) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_DEPOSIT_UPWARD;
      trajSetPose(withdraw_above[side]);
      *finish_tick = arm.trajStart();
    }
  }
  // 升高至存矿点上方
  else if (step == TRAJ_DEPOSIT_UPWARD) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_RELAY_1_BACK;
      trajSetPose(relay1[side]);
      *finish_tick = arm.trajStart();
    }
  }
  // 移动至兑换默认位姿
  else if (step == TRAJ_RELAY_1_BACK) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_EXCHANGE;
      trajSetPose(exchange_default);
      *finish_tick = arm.trajStart() + 500;
    }
  }
  // 移动至兑换默认位姿
  else if (step == TRAJ_EXCHANGE) {
    if (HAL_GetTick() > *finish_tick) {
      arm.trajAbort();
    }
  }
}

// 兑换处理
void ArmTask::Exchange_t::handle(void) {
  if (state == IDLE) {
    return;
  }

  // 准备阶段
  if (step == PREPARE) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_DEFAULT;
      trajSetPose(default_pose);
      *finish_tick = arm.trajStart() + 500;
    }
  }
  // 移动至默认位置
  else if (step == TRAJ_DEFAULT) {
    if (HAL_GetTick() > *finish_tick) {
      step = LOCATE;
      *finish_tick = UINT32_MAX;  // 等待取矿开始函数
    }
  }
  // 定位
  else if (step == LOCATE) {
    if (HAL_GetTick() > *finish_tick) {
      step = PUMP_E_OFF;
      pump_e.setValve(Pump::ValveState_e::OPEN_0);
      pump_e.setMotorSpeed(0);
      *finish_tick = HAL_GetTick() + 500;
    }
  }
  // 开启机械臂气泵
  else if (step == PUMP_E_OFF) {
    if (HAL_GetTick() > *finish_tick) {
      step = TRAJ_ROTATE;
      trajSetPose(rotate_pose);
      *finish_tick = arm.trajStart();
    }
  }
  // 末端旋转
  else if (step == TRAJ_ROTATE) {
    if (HAL_GetTick() > *finish_tick) {
      arm.trajAbort();
    }
  }
}

// 三连取矿处理
void ArmTask::triplePickHandle(void) {
  if (triple_pick_.state == IDLE) {
    return;
  }

  // 取1
  if (triple_pick_.step == TriplePick_t::PICK_1) {
    if (pick_.step == Pick_t::TRAJ_PICK_UPWARD &&
        HAL_GetTick() > finish_tick_) {
      // 设置为存矿模式
      triple_pick_.step = TriplePick_t::DEPOSIT_1;
      mode_ = DEPOSIT_0;
      deposit_.state = WORKING;
      deposit_.step = Deposit_t::PREPARE;
      deposit_.side = 0;
      finish_tick_ = HAL_GetTick() + 100;
      // 重置任务状态
      move_.state = IDLE;
      pick_.state = IDLE;
      // deposit_.state = IDLE;
      withdraw_.state = IDLE;
      exchange_.state = IDLE;
      // triple_pick_.state = IDLE;
    }
  }
  // 存1
  else if (triple_pick_.step == TriplePick_t::DEPOSIT_1) {
    if (deposit_.step == Deposit_t::TRAJ_DEPOSIT_UPWARD &&
        HAL_GetTick() > finish_tick_) {
      // 设置为取矿模式(跳过准备和移至默认步骤)
      triple_pick_.step = TriplePick_t::PICK_2;
      mode_ = PICK_NORMAL;
      pick_.state = WORKING;
      pick_.step = Pick_t::PUMP_E_ON;
      pick_.type = 0;
      pick_.start_pose = triple_pick_.mine[1];
      pick_.pick_up_pose = pick_.start_pose;
      pick_.pick_up_pose.z = pick_.start_pose.z + pick_.pick_up_dist;
      trajSetPose(pick_.start_pose);
      finish_tick_ = arm.trajStart() + 500;
      // 重置任务状态
      move_.state = IDLE;
      // pick_.state = IDLE;
      deposit_.state = IDLE;
      withdraw_.state = IDLE;
      exchange_.state = IDLE;
      // triple_pick_.state = IDLE;
    }
  }
  // 取2
  else if (triple_pick_.step == TriplePick_t::PICK_2) {
    if (pick_.step == Pick_t::TRAJ_PICK_UPWARD &&
        HAL_GetTick() > finish_tick_) {
      // 设置为存矿模式
      triple_pick_.step = TriplePick_t::DEPOSIT_2;
      mode_ = DEPOSIT_1;
      deposit_.state = WORKING;
      deposit_.step = Deposit_t::PREPARE;
      deposit_.side = 1;
      finish_tick_ = HAL_GetTick() + 100;
      // 重置任务状态
      move_.state = IDLE;
      pick_.state = IDLE;
      // deposit_.state = IDLE;
      withdraw_.state = IDLE;
      exchange_.state = IDLE;
      // triple_pick_.state = IDLE;
    }
  }
  // 存2
  else if (triple_pick_.step == TriplePick_t::DEPOSIT_2) {
    if (deposit_.step == Deposit_t::TRAJ_DEPOSIT_UPWARD &&
        HAL_GetTick() > finish_tick_) {
      // 设置为取矿模式(跳过准备和移至默认步骤)
      triple_pick_.step = TriplePick_t::PICK_3;
      mode_ = PICK_NORMAL;
      pick_.state = WORKING;
      pick_.step = Pick_t::PUMP_E_ON;
      pick_.type = 0;
      pick_.start_pose = triple_pick_.mine[2];
      pick_.pick_up_pose = pick_.start_pose;
      pick_.pick_up_pose.z = pick_.start_pose.z + pick_.pick_up_dist;
      trajSetPose(pick_.start_pose);
      finish_tick_ = arm.trajStart() + 500;
      // 重置任务状态
      move_.state = IDLE;
      // pick_.state = IDLE;
      deposit_.state = IDLE;
      withdraw_.state = IDLE;
      exchange_.state = IDLE;
      // triple_pick_.state = IDLE;
    }
  }
  // 取3
  else if (triple_pick_.step == TriplePick_t::PICK_3) {
    if (pick_.step == Pick_t::TRAJ_PICK_UPWARD &&
        HAL_GetTick() > finish_tick_) {
      triple_pick_.state = IDLE;
    }
  }
}

// 板载LED指示灯效果
void boardLedHandle(void) {
#ifdef DBC
  if (robot_state == STOP) {
    led.setColor(255, 0, 0);  // red
    led.setModeOn();
  } else if (robot_state == STARTUP) {
    led.setColor(150, 150, 0);  // yellow
    led.setModeBreath();
  } else if (robot_state == WORKING) {
    led.setColor(0, 0, 255);  // blue
    if (rc.switch_.l == RC::UP && rc.switch_.r == RC::UP) {
      // 遥控器挡位左上右上
      led.setModeBlink(1);
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::UP) {
      // 遥控器挡位左中右上
      led.setModeBlink(2);
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::UP) {
      // 遥控器挡位左下右上
      led.setModeBlink(3);
    } else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
      // 遥控器挡位左上右中
      led.setModeBlink(4);
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::MID) {
      // 遥控器挡位左中右中
      led.setModeBlink(5);
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::MID) {
      // 遥控器挡位左下右中
      led.setModeBlink(6);
    }
  }
  led.handle();
#elif defined DBA
  if (robot_state == STOP) {
    led.setLED(true, false, 0);  // red
  } else if (robot_state == STARTUP) {
    led.setLED(true, true, 0);  // red+green
  } else if (robot_state == WORKING) {
    if (rc.switch_.l == RC::UP && rc.switch_.r == RC::UP) {
      // 遥控器挡位左上右上
      led.setLED(false, true, 1);  // green
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::UP) {
      // 遥控器挡位左中右上
      led.setLED(false, true, 2);  // green
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::UP) {
      // 遥控器挡位左下右上
      led.setLED(false, true, 3);  // green
    } else if (rc.switch_.l == RC::UP && rc.switch_.r == RC::MID) {
      // 遥控器挡位左上右中
      led.setLED(false, true, 4);  // green
    } else if (rc.switch_.l == RC::MID && rc.switch_.r == RC::MID) {
      // 遥控器挡位左中右中
      led.setLED(false, true, 5);  // green
    } else if (rc.switch_.l == RC::DOWN && rc.switch_.r == RC::MID) {
      // 遥控器挡位左下右中
      led.setLED(false, true, 6);  // green
    }
  }
#endif
}
