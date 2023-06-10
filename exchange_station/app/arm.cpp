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

// 构造函数
Arm::Arm(Motor* j1, Motor* j2, Motor* j3, Motor* j4, Motor* j5, Motor* j6,
         Motor* j3_sup)
    : j1_(j1),
      j2_(j2),
      j3_(j3),
      j4_(j4),
      j5_(j5),
      j6_(j6),
      j3_sup_(j3_sup),
      arm_(links_) {
  init();
}

// 初始化关节角度
void Arm::init(void) {
  // todo
  float q[6] = {0, 0, 0, 0, 0, 0};
  fdb_.q = Matrixf<6, 1>(q);
  fdb_.q_D1 = matrixf::zeros<6, 1>();

  fdb_.T = arm_.fkine(fdb_.q);
  Matrixf<3, 1> p_fdb = robotics::t2p(fdb_.T);
  fdb_.x = p_fdb[0][0];
  fdb_.y = p_fdb[1][0];
  fdb_.z = p_fdb[2][0];
  Matrixf<3, 1> rpy_fdb = robotics::t2rpy(fdb_.T);
  fdb_.yaw = rpy_fdb[0][0];
  fdb_.pitch = rpy_fdb[1][0];
  fdb_.roll = rpy_fdb[2][0];

  ref_.q = fdb_.q;
  ref_.T = fdb_.T;
  ref_.x = fdb_.x;
  ref_.y = fdb_.y;
  ref_.z = fdb_.z;
  ref_.yaw = fdb_.yaw;
  ref_.pitch = fdb_.pitch;
  ref_.roll = fdb_.roll;
}

// 设置目标状态
void Arm::setRef(float x, float y, float z, float yaw, float pitch,
                 float roll) {
  ref_.x = x;
  ref_.y = y;
  ref_.z = z;
  ref_.yaw = yaw;
  ref_.pitch = pitch;
  ref_.roll = roll;
}

// 增量设置目标状态
void Arm::addRef(float x, float y, float z, float yaw, float pitch,
                 float roll) {
  ref_.x += x;
  ref_.y += y;
  ref_.z += z;
  ref_.yaw += yaw;
  ref_.pitch += pitch;
  ref_.roll += roll;
}

// 反馈状态解算，目标状态处理，运行控制器
void Arm::handle(void) {
  // 目标状态限制
  ref_.x = math::limit(ref_.x, 0.2f, 0.8f);
  ref_.y = math::limit(ref_.y, -0.3f, 0.3f);
  ref_.z = math::limit(ref_.z, 0.5f, 1.0f);
  ref_.yaw = math::limit(ref_.yaw, -PI / 2, PI / 2);
  ref_.pitch = math::limit(ref_.pitch, -60.0f, 0);
  ref_.roll = math::limit(ref_.roll, -45.0f, 45.0f);

  // 获取关节角度

  // test
  float q[6] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  float qv[6] = {1, 0.5, -1, 0.3, 0, -1};
  float qa[6] = {0.2, -0.3, 0.1, 0, -1, 0};
  float he[6] = {0};
  fdb_.T = arm_.fkine(q);
  fdb_.J = arm_.jacob(q);
  fdb_.q = ikine(fdb_.T);
  torq_ = arm_.rne(q, qv, qa, he);
}

// 逆运动学求解(解析形式)
Matrixf<6, 1> Arm::ikine(Matrixf<4, 4> T) {
  Matrixf<6, 1> q;
  // (1)
  Matrixf<3, 3> R06 = robotics::t2r(T);
  Matrixf<3, 1> o6 = robotics::t2p(T);
  Matrixf<3, 1> z6 = T.block<3, 1>(0, 2);
  // (2)
  Matrixf<3, 1> z5 = z6;
  Matrixf<3, 1> o5 = o6 - links_[5].dh_.d * z5;
  // (3)
  float d234 = links_[1].dh_.d + links_[2].dh_.d + links_[3].dh_.d;
  float xo5 = o5[0][0];
  float yo5 = o5[1][0];
  if (fabs(d234) < 1e-8f) {
    q[0][0] = atan2f(yo5, xo5);
  } else {
    q[0][0] = atan2f(yo5, xo5) +
              acosf(fabs(d234) / sqrtf(xo5 * xo5 + yo5 * yo5)) +
              fabs(d234) / d234 * PI / 2;
  }
  // (4)
  float r01[9] = {cosf(q[0][0]),
                  0,
                  sinf(q[0][0]),
                  sinf(q[0][0]),
                  0,
                  -cos(q[0][0]),
                  0,
                  1,
                  0};
  Matrixf<3, 3> R01(r01);
  Matrixf<3, 1> z1 = R01.block<3, 1>(0, 2);
  Matrixf<3, 1> z4_raw = vector3f::cross(z1, z5);
  static Matrixf<3, 1> z4;
  if (z4_raw.norm() > 1e-8f) {
    z4 = z4_raw / z4_raw.norm();
  }
  Matrixf<3, 1> o4 = o5 - links_[4].dh_.d * z4;
  // (5)
  Matrixf<3, 1> o1 = matrixf::zeros<3, 1>();
  o1[2][0] = links_[0].dh_.d;
  Matrixf<3, 1> p14 = o4 - o1;
  float a2 = links_[1].dh_.a;
  float a3 = links_[2].dh_.a;
  float phi =
      acosf((a2 * a2 + a3 * a3 + d234 * d234 - p14.norm() * p14.norm()) /
            (2 * a2 * a3));
  q[2][0] = phi - PI / 2;
  // (6)
  Matrixf<3, 1> y1 = matrixf::zeros<3, 1>();
  y1[2][0] = 1;
  Matrixf<3, 1> x1 = vector3f::cross(y1, z1);
  float gamma =
      asinf(a3 * sinf(phi) / sqrtf(p14.norm() * p14.norm() - d234 * d234));
  q[1][0] = atan2f((y1.trans() * p14)[0][0], (x1.trans() * p14)[0][0]) + gamma -
            PI / 2;
  // (7)
  Matrixf<3, 1> y4 = matrixf::zeros<3, 1>() - z1;
  Matrixf<3, 1> x4 = vector3f::cross(y4, z4);
  float r04[9] = {x4[0][0], y4[0][0], z4[0][0], x4[1][0], y4[1][0],
                  z4[1][0], x4[2][0], y4[2][0], z4[2][0]};
  Matrixf<3, 3> R04 = Matrixf<3, 3>(r04);
  float r13[9] = {cosf(q[1][0] + q[2][0]),
                  -sinf(q[1][0] + q[2][0]),
                  0,
                  sinf(q[1][0] + q[2][0]),
                  cosf(q[1][0] + q[2][0]),
                  0,
                  0,
                  0,
                  1};
  Matrixf<3, 3> R13 = Matrixf<3, 3>(r13);
  Matrixf<3, 3> R34 = (R01 * R13).trans() * R04;
  q[3][0] = atan2f(R34[1][0], R34[0][0]);
  // (8)
  Matrixf<3, 1> y5 = z4;
  Matrixf<3, 1> x5 = vector3f::cross(y5, z5);
  float r05[9] = {x5[0][0], y5[0][0], z5[0][0], x5[1][0], y5[1][0],
                  z5[1][0], x5[2][0], y5[2][0], z5[2][0]};
  Matrixf<3, 3> R05 = Matrixf<3, 3>(r05);
  Matrixf<3, 3> R45 = R04.trans() * R05;
  q[4][0] = atan2f(R45[1][0], R45[0][0]) - PI / 2;
  // (9)
  Matrixf<3, 3> R56 = R05.trans() * R06;
  q[5][0] = atan2f(R56[1][0], R56[0][0]);

  return q;
}

// 操作空间控制器(末端位姿)
void Arm::manipulationController(void) {}

// 关节空间控制器(关节角度)
void Arm::jointController(void) {}

// 停止状态控制器(电机断电/阻尼模式)
void Arm::stopController(void) {
  // 重置目标状态
}
