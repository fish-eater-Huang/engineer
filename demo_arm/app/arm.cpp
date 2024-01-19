/**
 ******************************************************************************
 * @file    arm.cpp/h
 * @brief   mechanical arm control. 机械臂控制
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "app/arm.h"
#include "base/common/math.h"
#include "tim.h"

// 构造函数
Arm::Arm(Motor* j1, Motor* j2, Motor* j3, Motor* j4, Motor* j5, Motor* j6,
         IMU* imu0, IMU* imu2, IMU* imu3, ImuComm* imu_comm)
    : j1_(j1), j2_(j2), j3_(j3), j4_(j4), j5_(j5), j6_(j6), arm_(links_) {
  init_.is_finish = false;
  init_.method = Arm::Init_t::Method_e::MANUAL;
  init_.imu0 = imu0;
  init_.imu2 = imu2;
  init_.imu3 = imu3;
  init_.imu_comm = imu_comm;
  init_.imu2_connect = Connect(1000);
  init_.imu3_connect = Connect(1000);
}

// 初始化关节角度
void Arm::init(void) {
  // 初始角度设置
  // 手动初始化（移至初始化位置开机）
  float q_init_deg[6] = {0, -180.0f, 80.0f, 0, 100.0f, 0};
  if (init_.method == Arm::Init_t::Method_e::MANUAL) {
    j1_->resetFeedbackAngle(q_init_deg[0]);
    j2_->resetFeedbackAngle(q_init_deg[1]);
    j3_->resetFeedbackAngle(q_init_deg[2]);
    j4_->resetFeedbackAngle(q_init_deg[3]);
    j5_->resetFeedbackAngle(q_init_deg[4]);
    j6_->resetFeedbackAngle(q_init_deg[5]);
  }
  // 使用IMU初始化关节电机角度
  else if (init_.method == Arm::Init_t::Method_e::LINK_IMU) {
    // 等待陀螺仪上电 & 重力补偿收敛
    if (HAL_GetTick() < 2000) {
      return;
    }
    // 检测陀螺仪连接状态 todo
    if (!(init_.imu_comm->imu2_connect_.check() &&
          init_.imu_comm->imu1_connect_.check()) ||
        !(fabs(init_.imu0->pitch()) < 10.0f &&
          fabs(init_.imu0->roll()) < 10.0f)) {
      // 若未检测到陀螺仪或link0陀螺仪不水平则切换为手动初始化
      init_.method = Arm::Init_t::Method_e::MANUAL;
      return;
    }

    // 根据陀螺仪角度初始化关节电机角度
    float rpy2[3] = {math::deg2rad(init_.imu2->yaw()),
                     math::deg2rad(init_.imu2->pitch()),
                     math::deg2rad(init_.imu2->roll())};
    float rpy3[3] = {math::deg2rad(init_.imu3->yaw()),
                     math::deg2rad(init_.imu3->pitch()),
                     math::deg2rad(init_.imu3->roll())};
    Matrixf<3, 3> Rw2 = robotics::rpy2r(rpy2);
    Matrixf<3, 3> Rw3 = robotics::rpy2r(rpy3);
    q_init_deg[1] = math::rad2deg(
        math::loopLimit(atan2f(Rw2[2][0], -Rw2[2][2]), -PI * 1.5f, PI * 0.5f));
    q_init_deg[2] = math::rad2deg(math::loopLimit(
        atan2f(-Rw3[2][2], -Rw3[2][0]) - math::deg2rad(q_init_deg[1]), -PI,
        PI));

    // 设置电机反馈角度
    j1_->resetFeedbackAngle(q_init_deg[0]);
    j2_->resetFeedbackAngle(q_init_deg[1]);
    j3_->resetFeedbackAngle(q_init_deg[2]);
    j4_->resetFeedbackAngle(q_init_deg[3]);
    j5_->resetFeedbackAngle(q_init_deg[4]);
    j6_->resetFeedbackAngle(q_init_deg[5]);
  }
  // 使用编码器初始化关节电机角度
  else if (init_.method == Arm::Init_t::Method_e::ENCODER) {
    // todo
  }

  // 更新反馈状态
  float q[6];
  for (int i = 0; i < 6; i++) {
    q[i] = math::deg2rad(q_init_deg[i]);
  }
  fdb_.q = Matrixf<6, 1>(q);
  fdb_.q_D1 = matrixf::zeros<6, 1>();
  fdb_.T = arm_.fkine(fdb_.q);
  R0_ = robotics::t2r(arm_.fkine(matrixf::zeros<6, 1>()));
  float r0[9] = {0, 0, 1, 0, -1, 0, 1, 0, 0};
  R0_ = Matrixf<3, 3>(r0);
  Matrixf<3, 1> p_fdb = robotics::t2p(fdb_.T);
  fdb_.x = p_fdb[0][0];
  fdb_.y = p_fdb[1][0];
  fdb_.z = p_fdb[2][0];
  Matrixf<3, 1> rpy_fdb = robotics::r2rpy(robotics::t2r(fdb_.T) * R0_.trans());
  fdb_.yaw = rpy_fdb[0][0];
  fdb_.pitch = rpy_fdb[1][0];
  fdb_.roll = rpy_fdb[2][0];

  // 更新目标状态
  ref_.q = fdb_.q;
  ref_.T = fdb_.T;
  ref_.x = fdb_.x;
  ref_.y = fdb_.y;
  ref_.z = fdb_.z;
  ref_.yaw = fdb_.yaw;
  ref_.pitch = fdb_.pitch;
  ref_.roll = fdb_.roll;

  // 设置电机反馈数据源
  j1_->setFdbSrc(&j1_->kfAngle(), &j1_->kfSpeed());
  j2_->setFdbSrc(&j2_->kfAngle(), &j2_->kfSpeed());
  j3_->setFdbSrc(&j3_->kfAngle(), &j3_->kfSpeed());
  j4_->setFdbSrc(&j4_->kfAngle(), &j4_->kfSpeed());
  j5_->setFdbSrc(&j5_->kfAngle(), &j5_->kfSpeed());
  j6_->setFdbSrc(&j6_->kfAngle(), &j6_->kfSpeed());

  // 力矩前馈清零
  torq_ = matrixf::zeros<6, 1>();

  // 设置初始目标角度
  j1_->setAngleSpeed(math::rad2deg(ref_.q[0][0]), 0, torq_[0][0]);
  j2_->setAngleSpeed(math::rad2deg(ref_.q[1][0]), 0, torq_[1][0]);
  j3_->setAngleSpeed(math::rad2deg(ref_.q[2][0]), 0, torq_[2][0]);
  j4_->setAngleSpeed(math::rad2deg(ref_.q[3][0]), 0, torq_[3][0]);
  j5_->setAngleSpeed(math::rad2deg(ref_.q[4][0]), 0, torq_[4][0]);
  j6_->setAngleSpeed(math::rad2deg(ref_.q[5][0]), 0, torq_[5][0]);

  init_.is_finish = true;
}

// 反馈状态解算，目标状态处理，运行控制器
void Arm::handle(void) {
  // 检测电机连接状态
  if (!(j1_->connect_.check() && j2_->connect_.check() &&
        j3_->connect_.check() && j4_->connect_.check() &&
        j5_->connect_.check() && j6_->connect_.check())) {
    // 电机离线重新初始化
    init_.is_finish = false;
  }



  // 初始化处理
  if (!init_.is_finish) {
    init();
    return;
  }

  // 获取关节角度&角速度
  fdb_.q[0][0] = math::deg2rad(j1_->realAngle());
  fdb_.q[1][0] = math::deg2rad(j2_->realAngle());
  fdb_.q[2][0] = math::deg2rad(j3_->realAngle());
  fdb_.q[3][0] = math::deg2rad(j4_->realAngle());
  fdb_.q[4][0] = math::deg2rad(j5_->realAngle());
  fdb_.q[5][0] = math::deg2rad(j6_->realAngle());
  fdb_.q_D1[0][0] = math::dps2radps(j1_->realSpeed());
  fdb_.q_D1[1][0] = math::dps2radps(j2_->realSpeed());
  fdb_.q_D1[2][0] = math::dps2radps(j3_->realSpeed());
  fdb_.q_D1[3][0] = math::dps2radps(j4_->realSpeed());
  fdb_.q_D1[4][0] = math::dps2radps(j5_->realSpeed());
  fdb_.q_D1[5][0] = math::dps2radps(j6_->realSpeed());

  // 反馈位姿解算(正运动学)
  fdb_.T = arm_.fkine(fdb_.q);
  fdb_.J = arm_.jacob(fdb_.q);
  Matrixf<3, 1> p_fdb = robotics::t2p(fdb_.T);
  fdb_.x = p_fdb[0][0];
  fdb_.y = p_fdb[1][0];
  fdb_.z = p_fdb[2][0];
  Matrixf<3, 1> rpy_fdb = robotics::r2rpy(robotics::t2r(fdb_.T) * R0_.trans());
  fdb_.yaw = rpy_fdb[0][0];
  fdb_.pitch = rpy_fdb[1][0];
  fdb_.roll = rpy_fdb[2][0];

  // 控制器
  if (mode_ == Arm::Mode_e::STOP) {
    stopController();
  } else if (mode_ == Arm::Mode_e::MANIPULATION) {
    trajectoryPlanner();
    manipulationController();
  } else if (mode_ == Arm::Mode_e::JOINT) {
    trajectoryPlanner();
    jointController();
  } else if (mode_ == Arm::Mode_e::COMPLIANCE) {
    complianceController();
  }
}

// 设置目标状态
void Arm::setRef(const float& x, const float& y, const float& z,
                 const float& yaw, const float& pitch, const float& roll) {
  ref_.x = x;
  ref_.y = y;
  ref_.z = z;
  ref_.yaw = yaw;
  ref_.pitch = pitch;
  ref_.roll = roll;
}

// 增量设置目标状态
void Arm::addRef(const float& x, const float& y, const float& z,
                 const float& yaw, const float& pitch, const float& roll) {
  ref_.x += x;
  ref_.y += y;
  ref_.z += z;
  ref_.yaw += yaw;
  ref_.pitch += pitch;
  ref_.roll += roll;
}

// 设置关节目标状态
void Arm::setJointRef(const float& q1, const float& q2, const float& q3,
                      const float& q4, const float& q5, const float& q6) {
  ref_.q[0][0] = q1;
  ref_.q[1][0] = q2;
  ref_.q[2][0] = q3;
  ref_.q[3][0] = q4;
  ref_.q[4][0] = q5;
  ref_.q[5][0] = q6;
}

// 增量设置关节目标状态
void Arm::addJointRef(const float& q1, const float& q2, const float& q3,
                      const float& q4, const float& q5, const float& q6) {
  ref_.q[0][0] += q1;
  ref_.q[1][0] += q2;
  ref_.q[2][0] += q3;
  ref_.q[3][0] += q4;
  ref_.q[4][0] += q5;
  ref_.q[5][0] += q6;
}

// 逆运动学求解(解析形式)
Matrixf<6, 1> Arm::ikine(Matrixf<4, 4> T, Matrixf<6, 1> q0) {
  Matrixf<6, 1> q = q0;

  // (1)
  Matrixf<3, 3> R06 = robotics::t2r(T);
  Matrixf<3, 1> o6 = robotics::t2p(T);
  Matrixf<3, 1> z6 = T.block<3, 1>(0, 2);

  // (2)
  Matrixf<3, 1> z5 = z6;
  Matrixf<3, 1> o5 = o6 - links_[5].dh_.d * z5;

  // 处理奇异输入(肘部奇异&肩部奇异)
  float a2 = links_[1].dh_.a;
  float d2 = links_[1].dh_.d;
  float d4 = links_[3].dh_.d;
  if (o5.norm() > (a2 + d4) * 0.9f) {
    o5 = o5 / o5.norm() * (a2 + d4) * 0.9f;
  } else if (sqrtf(o5[0][0] * o5[0][0] + o5[1][0] * o5[1][0]) <
             (a2 + d4) * 0.15f) {
    o5[0][0] = o5[0][0] / sqrtf(o5[0][0] * o5[0][0] + o5[1][0] * o5[1][0]) *
               (a2 + d4) * 0.15f;
    o5[1][0] = o5[1][0] / sqrtf(o5[0][0] * o5[0][0] + o5[1][0] * o5[1][0]) *
               (a2 + d4) * 0.15f;
  }

  // (3)-θ1
  float xo5 = o5[0][0];
  float yo5 = o5[1][0];
  q[0][0] = atan2f(yo5, xo5);

  // (4)-θ3
  Matrixf<3, 1> o1 = matrixf::zeros<3, 1>();
  o1[2][0] = links_[0].dh_.d;
  Matrixf<3, 1> p15 = o5 - o1;
  float phi = acosf((a2 * a2 + d4 * d4 + d2 * d2 - p15.norm() * p15.norm()) /
                    (2 * a2 * d4));
  q[2][0] = PI / 2 - phi;

  // (5)-θ2
  Matrixf<3, 1> y1 = matrixf::zeros<3, 1>();
  y1[2][0] = -1;
  Matrixf<3, 1> x1 = matrixf::zeros<3, 1>();
  x1[0][0] = cosf(q[0][0]);
  x1[1][0] = sinf(q[0][0]);
  float gamma =
      asinf(d4 * sinf(phi) / sqrtf(p15.norm() * p15.norm() - d2 * d2));
  q[1][0] = atan2f((y1.trans() * p15)[0][0], (x1.trans() * p15)[0][0]) - gamma;

  // (6)
  float r01[9] = {cosf(q[0][0]),
                  0,
                  -sinf(q[0][0]),
                  sinf(q[0][0]),
                  0,
                  cos(q[0][0]),
                  0,
                  -1,
                  0};
  float r12[9] = {cosf(q[1][0]),
                  -sinf(q[1][0]),
                  0,
                  sinf(q[1][0]),
                  cos(q[1][0]),
                  0,
                  0,
                  0,
                  1};
  float r23[9] = {cosf(q[2][0]),
                  0,
                  -sinf(q[2][0]),
                  sinf(q[2][0]),
                  0,
                  cos(q[2][0]),
                  0,
                  -1,
                  0};
  Matrixf<3, 3> R01(r01), R12(r12), R23(r23);
  Matrixf<3, 3> R36 = (R01 * R12 * R23).trans() * R06;

  // (7)-θ4
  // 若无腕部奇异则更新θ4，否则保留θ4的值
  if (R36.block<2, 1>(0, 2).norm() > 0.1f) {
    float theta4[2];
    theta4[0] = math::radNormalizePI(atan2f(R36[1][2], R36[0][2]));
    theta4[1] = math::radNormalizePI(atan2f(R36[1][2], R36[0][2]) + PI);
    if (fabs(theta4[0] - q[3][0]) < fabs(theta4[1] - q[3][0])) {
      q[3][0] = theta4[0];
    } else {
      q[3][0] = theta4[1];
    }
  }

  // (8)-θ5
  q[4][0] = atan2f(-math::sign(cosf(q[3][0])) * math::sign(R36[0][2]) *
                       R36.block<2, 1>(0, 2).norm(),
                   R36[2][2]);

  // (9)-θ6
  float r34[9] = {cosf(q[3][0]),
                  0,
                  sinf(q[3][0]),
                  sinf(q[3][0]),
                  0,
                  -cos(q[3][0]),
                  0,
                  1,
                  0};
  float r45[9] = {cosf(q[4][0]),
                  0,
                  -sinf(q[4][0]),
                  sinf(q[4][0]),
                  0,
                  cos(q[4][0]),
                  0,
                  -1,
                  0};
  Matrixf<3, 3> R34(r34), R45(r45);
  Matrixf<3, 3> R56 = (R34 * R45).trans() * R36;
  q[5][0] = atan2f(R56[1][0], R56[0][0]);

  return q;
}

// 停止状态控制器(电机断电/阻尼模式)
void Arm::stopController(void) {
  // 重置目标状态
  ref_.q = fdb_.q;
  ref_.T = fdb_.T;
  ref_.x = fdb_.x;
  ref_.y = fdb_.y;
  ref_.z = fdb_.z;
  ref_.yaw = fdb_.yaw;
  ref_.pitch = fdb_.pitch;
  ref_.roll = fdb_.roll;

  // 力矩前馈清零
  torq_ = matrixf::zeros<6, 1>();
}

// 操作空间控制器(末端位姿)
void Arm::manipulationController(void) {
  // 目标状态限制
  Matrixf<3, 1> z60 = fdb_.T.block<3, 1>(0, 2);
  Matrixf<3, 1> p56 = links_[5].dh_.d * z60;
  float x56 = p56[0][0];
  float y56 = p56[1][0];
  float z56 = p56[2][0];
  // 位置边界限制
  float a2d4 = links_[1].dh_.a + links_[3].dh_.d;
  ref_.x = math::limit(ref_.x, -a2d4 * 0.9f + x56, a2d4 * 0.9f + x56);
  ref_.y = math::limit(ref_.y, -a2d4 * 0.9f + y56, a2d4 * 0.9f + y56);
  ref_.z = math::limit(ref_.z, -a2d4 * 0.9f + z56, a2d4 * 0.9f + z56);
  // 肘部奇异限位
  float ref_p[3] = {ref_.x, ref_.y, ref_.z};
  float o5_norm = (Matrixf<3, 1>(ref_p) - p56).norm();
  if (o5_norm > a2d4 * 0.9f) {
    ref_.x = (ref_.x - x56) / o5_norm * a2d4 * 0.9f + x56;
    ref_.y = (ref_.y - y56) / o5_norm * a2d4 * 0.9f + y56;
    ref_.z = (ref_.z - z56) / o5_norm * a2d4 * 0.9f + z56;
  }
  // 肩部奇异限位
  float o5_rxy = (Matrixf<3, 1>(ref_p) - p56).block<2, 1>(0, 0).norm();
  if (o5_rxy < a2d4 * 0.15f) {
    ref_.x = (ref_.x - x56) / o5_rxy * a2d4 * 0.15f + x56;
    ref_.y = (ref_.y - y56) / o5_rxy * a2d4 * 0.15f + y56;
  }
  // 位置差值限制
  ref_.x = math::limit(ref_.x, fdb_.x - 0.1f, fdb_.x + 0.1f);
  ref_.y = math::limit(ref_.y, fdb_.y - 0.1f, fdb_.y + 0.1f);
  ref_.z = math::limit(ref_.z, fdb_.z - 0.1f, fdb_.z + 0.1f);
  // J1边界限制
  if (fdb_.x - x56 < 0.02f && fdb_.y - y56 >= 0) {
    ref_.y = math::limitMin(ref_.y, 0.02f + y56);
  } else if (fdb_.x - x56 < 0.02f && fdb_.y - y56 < 0) {
    ref_.y = math::limitMax(ref_.y, -0.02f + y56);
  }
  //  ref_.yaw = math::limit(ref_.yaw, -PI / 2, PI / 2);
  //  ref_.pitch = math::limit(ref_.pitch, -60.0f, 0);
  //  ref_.roll = math::limit(ref_.roll, -45.0f, 45.0f);

  // 目标状态解算
  Matrixf<3, 1> p_ref;
  p_ref[0][0] = ref_.x;
  p_ref[1][0] = ref_.y;
  p_ref[2][0] = ref_.z;
  Matrixf<3, 1> rpy_ref;
  rpy_ref[0][0] = ref_.yaw;
  rpy_ref[1][0] = ref_.pitch;
  rpy_ref[2][0] = ref_.roll;
  ref_.T = robotics::rp2t(robotics::rpy2r(rpy_ref) * R0_, p_ref);

  // 逆运动学解析解
  ref_.q = ikine(ref_.T, fdb_.q);
  // ref_.q[5][0] =
  //     math::loopLimit(ref_.q[5][0], fdb_.q[5][0] - PI, fdb_.q[5][0] + PI);

  // 自重补偿作为动力学前馈(取动力学方程位置项)
  torq_ = arm_.rne(fdb_.q);

  // 电机控制
  j1_->method_ = Motor::ControlMethod_e::POSITION_SPEED;
  j2_->method_ = Motor::ControlMethod_e::POSITION_SPEED;
  j3_->method_ = Motor::ControlMethod_e::POSITION_SPEED;
  j4_->method_ = Motor::ControlMethod_e::POSITION_SPEED;
  j5_->method_ = Motor::ControlMethod_e::POSITION_SPEED;
  j6_->method_ = Motor::ControlMethod_e::POSITION_SPEED;
  j1_->setAngleSpeed(math::rad2deg(ref_.q[0][0]), 0, torq_[0][0]);
  j2_->setAngleSpeed(math::rad2deg(ref_.q[1][0]), 0, torq_[1][0]);
  j3_->setAngleSpeed(math::rad2deg(ref_.q[2][0]), 0, torq_[2][0]);
  j4_->setAngleSpeed(math::rad2deg(ref_.q[3][0]), 0, torq_[3][0]);
  j5_->setAngleSpeed(math::rad2deg(ref_.q[4][0]), 0, torq_[4][0]);
  j6_->setAngleSpeed(math::rad2deg(ref_.q[5][0]), 0, torq_[5][0]);
}

// 关节空间控制器(关节角度)
void Arm::jointController(void) {
  // 关节限位
  ref_.q[0][0] = math::limit(ref_.q[0][0], -PI * 0.95f, PI * 0.95f);
  ref_.q[1][0] = math::limit(ref_.q[1][0], -PI, 0);
  ref_.q[2][0] = math::limit(ref_.q[2][0], -PI * 0.5f, math::deg2rad(80));
  ref_.q[3][0] = math::limit(ref_.q[3][0], -PI, PI);
  ref_.q[4][0] = math::limit(ref_.q[4][0], -PI, PI);
  ref_.q[5][0] = math::limit(ref_.q[5][0], -PI, PI);

  // 正运动学
  ref_.T = fdb_.T;
  ref_.x = fdb_.x;
  ref_.y = fdb_.y;
  ref_.z = fdb_.z;
  ref_.yaw = fdb_.yaw;
  ref_.pitch = fdb_.pitch;
  ref_.roll = fdb_.roll;

  // 电机控制
  j1_->method_ = Motor::ControlMethod_e::POSITION_SPEED;
  j2_->method_ = Motor::ControlMethod_e::POSITION_SPEED;
  j3_->method_ = Motor::ControlMethod_e::POSITION_SPEED;
  j4_->method_ = Motor::ControlMethod_e::POSITION_SPEED;
  j5_->method_ = Motor::ControlMethod_e::POSITION_SPEED;
  j6_->method_ = Motor::ControlMethod_e::POSITION_SPEED;
  j1_->setAngleSpeed(math::rad2deg(ref_.q[0][0]), 0, torq_[0][0]);
  j2_->setAngleSpeed(math::rad2deg(ref_.q[1][0]), 0, torq_[1][0]);
  j3_->setAngleSpeed(math::rad2deg(ref_.q[2][0]), 0, torq_[2][0]);
  j4_->setAngleSpeed(math::rad2deg(ref_.q[3][0]), 0, torq_[3][0]);
  j5_->setAngleSpeed(math::rad2deg(ref_.q[4][0]), 0, torq_[4][0]);
  j6_->setAngleSpeed(math::rad2deg(ref_.q[5][0]), 0, torq_[5][0]);
}

// 柔顺控制器
void Arm::complianceController(void) {
  // 重置目标状态
  ref_.q = fdb_.q;
  ref_.T = fdb_.T;
  ref_.x = fdb_.x;
  ref_.y = fdb_.y;
  ref_.z = fdb_.z;
  ref_.yaw = fdb_.yaw;
  ref_.pitch = fdb_.pitch;
  ref_.roll = fdb_.roll;

  // 自重补偿
  torq_ = arm_.rne(fdb_.q);

  // 电机控制
  j1_->method_ = Motor::ControlMethod_e::TORQUE;
  j2_->method_ = Motor::ControlMethod_e::TORQUE;
  j3_->method_ = Motor::ControlMethod_e::TORQUE;
  j4_->method_ = Motor::ControlMethod_e::TORQUE;
  j5_->method_ = Motor::ControlMethod_e::TORQUE;
  j6_->method_ = Motor::ControlMethod_e::TORQUE;
  j1_->targetTorque() = torq_[0][0];
  j2_->targetTorque() = torq_[1][0];
  j3_->targetTorque() = torq_[2][0];
  j4_->targetTorque() = torq_[3][0];
  j5_->targetTorque() = torq_[4][0];
  j6_->targetTorque() = torq_[5][0];
}

// 设置轨迹终点(末端位姿)+时间(ms)
void Arm::trajSet(const float& x, const float& y, const float& z,
                  const float& yaw, const float& pitch, const float& roll,
                  const float& speed, const float& rotate_speed) {
  // 设置轨迹终点位姿
  traj_.end.x = x;
  traj_.end.y = y;
  traj_.end.z = z;
  traj_.end.yaw = yaw;
  traj_.end.pitch = pitch;
  traj_.end.roll = roll;

  // 设置轨迹速度/角速度
  traj_.speed = fmaxf(fabs(speed), 1e-6f);
  traj_.rotate_speed = fmaxf(fabs(rotate_speed), 1e-6f);
}

// 设置轨迹终点(关节角度)+速度
void Arm::trajSet(Matrixf<6, 1> q, Matrixf<6, 1> q_D1) {
  // 设置轨迹终点关节角度
  traj_.end.q = q;

  // 设置关节角速度
  for (int i = 0; i < 6; i++) {
    traj_.q_D1[i][0] = fmaxf(fabs(q_D1[i][0]), 1e-6f);
  }
}

// 开始轨迹，返回轨迹结束时间
uint32_t Arm::trajStart(void) {
  // 设置轨迹起点为当前状态
  traj_.start.q = fdb_.q;
  traj_.start.x = fdb_.x;
  traj_.start.y = fdb_.y;
  traj_.start.z = fdb_.z;
  traj_.start.yaw = fdb_.yaw;
  traj_.start.pitch = fdb_.pitch;
  traj_.start.roll = fdb_.roll;

  // 设置轨迹开始时间为当前时间
  traj_.start.tick = HAL_GetTick();

  // 工作空间插值
  if (traj_.method == Arm::Traj_t::MANIPULATION) {
    // 位移时间
    float dx = traj_.end.x - traj_.start.x;
    float dy = traj_.end.y - traj_.start.y;
    float dz = traj_.end.z - traj_.start.z;
    if (traj_.speed == 0) {
      return 0;
    }
    float ticks_pos = sqrtf(dx * dx + dy * dy + dz * dz) / traj_.speed * 1e3f;

    // 旋转时间
    float rpy_start[3] = {traj_.start.yaw, traj_.start.pitch, traj_.start.roll};
    float rpy_end[3] = {traj_.end.yaw, traj_.end.pitch, traj_.end.roll};
    traj_.start.R = robotics::rpy2r(rpy_start);
    traj_.end.R = robotics::rpy2r(rpy_end);
    traj_.r_theta = robotics::r2angvec(traj_.start.R.trans() * traj_.end.R);
    if (traj_.rotate_speed == 0) {
      return 0;
    }
    float ticks_rot = fabs(traj_.r_theta[3][0]) / traj_.rotate_speed * 1e3f;

    // 取位移时间和旋转时间中较长的计算轨迹时间
    traj_.ticks = (uint32_t)fmax(ticks_pos, ticks_rot);
    traj_.end.tick = traj_.start.tick + traj_.ticks;
  }
  // 关节空间插值
  else if (traj_.method == Arm::Traj_t::JOINT) {
    // 关节运动时间
    Matrixf<6, 1> dq = traj_.end.q - traj_.start.q;
    Matrixf<6, 1> ticks = matrixf::zeros<6, 1>();
    traj_.ticks = 0;
    for (int i = 0; i < 6; i++) {
      if (traj_.q_D1[i][0] == 0) {
        return 0;
      }
      traj_.ticks = fmax(traj_.ticks, fabs(dq[i][0]) / traj_.q_D1[i][0] * 1e3f);
    }
    traj_.end.tick = traj_.start.tick + traj_.ticks;
  }

  // 设置轨迹规划状态
  traj_.state = true;
  return traj_.ticks;
}

// 中止轨迹
void Arm::trajAbort(void) {
  // 设置轨迹规划状态
  traj_.state = false;
}

// 轨迹规划处理
void Arm::trajectoryPlanner(void) {
  if (traj_.state) {
    float sigma = 1;
    if (traj_.ticks > 1) {
      sigma = (float)(HAL_GetTick() - traj_.start.tick) / (float)traj_.ticks;
    }
    traj_.sigma = math::limit(sigma, 0, 1);

    // 工作空间控制模式
    if (mode_ == Arm::Mode_e::MANIPULATION) {
      // 工作空间插值
      if (traj_.method == Arm::Traj_t::MANIPULATION) {
        // 末端位置线性插值
        ref_.x = traj_.sigma * traj_.end.x + (1 - traj_.sigma) * traj_.start.x;
        ref_.y = traj_.sigma * traj_.end.y + (1 - traj_.sigma) * traj_.start.y;
        ref_.z = traj_.sigma * traj_.end.z + (1 - traj_.sigma) * traj_.start.z;
        // 末端姿态Slerp插值
        float r_theta[4] = {traj_.r_theta[0][0], traj_.r_theta[1][0],
                            traj_.r_theta[2][0],
                            traj_.r_theta[3][0] * traj_.sigma};
        Matrixf<3, 1> rpy_ref =
            robotics::r2rpy(traj_.start.R * robotics::angvec2r(r_theta));
        ref_.yaw = rpy_ref[0][0];
        ref_.pitch = rpy_ref[1][0];
        ref_.roll = rpy_ref[2][0];
      }
      // 关节空间插值
      else if (traj_.method == Arm::Traj_t::JOINT) {
        // 轨迹规划直接设置关节角度
        ref_.q = traj_.sigma * traj_.end.q + (1 - traj_.sigma) * traj_.start.q;
        ref_.T = arm_.fkine(ref_.q);
        Matrixf<3, 1> p_ref = robotics::t2p(ref_.T);
        ref_.x = p_ref[0][0];
        ref_.y = p_ref[1][0];
        ref_.z = p_ref[2][0];
        Matrixf<3, 1> rpy_ref =
            robotics::r2rpy(robotics::t2r(ref_.T) * R0_.trans());
        ref_.yaw = rpy_ref[0][0];
        ref_.pitch = rpy_ref[1][0];
        ref_.roll = rpy_ref[2][0];
      }
    }
    // 关节空间控制模式
    else if (mode_ == Arm::Mode_e::JOINT) {
      if (traj_.method == Arm::Traj_t::JOINT) {
        // 关节角度线性插值
        ref_.q = traj_.sigma * traj_.end.q + (1 - traj_.sigma) * traj_.start.q;
      } else {
        trajAbort();
      }
    } else {
      trajAbort();
    }
  } else {
    traj_.sigma = 0;
  }
}
