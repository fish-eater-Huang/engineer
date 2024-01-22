//
// Created by 15795 on 2024/1/19.
//

#include "auto_exchange.h"
#include "base/cv_comm/cv_comm.h"
extern CVComm cv_comm;
extern Arm arm;
float test1,test2;
float r0[9] = {0, 0, 1, 0, -1, 0, 1, 0, 0};
Matrixf<3,3> R0_ = Matrixf<3, 3>(r0);

AutoExchangeController autoexchange_controller;

AutoExchangeController::AutoExchangeController()
{
    this->ref_.x = arm.fdb_.x;
    this->ref_.y = arm.fdb_.y;
    this->ref_.z = arm.fdb_.z;
    this->ref_.yaw = arm.fdb_.yaw;
    this->ref_.pitch = arm.fdb_.pitch;
    this->ref_.roll = arm.fdb_.roll;
}



Matrixf<4,4> AutoExchangeController::cv2t(void)
{
    //视觉传入的数据是位置和旋转向量，根据Rodrigues公式将其计算得到旋转矩阵。
    float theta = sqrtf(powf(cv_comm.auto_exchange_pc2board_msg_.RotX,2)+powf(cv_comm.auto_exchange_pc2board_msg_.RotY,2)+powf(cv_comm.auto_exchange_pc2board_msg_.RotZ,2));
    //定义旋转轴
    Matrixf<3,1>  Rotate_axis;
    //单位化
    Rotate_axis[0][0] = cv_comm.auto_exchange_pc2board_msg_.RotX/theta;
    Rotate_axis[1][0] = cv_comm.auto_exchange_pc2board_msg_.RotY/theta;
    Rotate_axis[2][0] = cv_comm.auto_exchange_pc2board_msg_.RotZ/theta;

    Matrixf<4,4> cv_t;
    Matrixf<3,1> p_cv_ref;
    p_cv_ref[0][0] = cv_comm.auto_exchange_pc2board_msg_.TransX;
    p_cv_ref[1][0] = cv_comm.auto_exchange_pc2board_msg_.TransY;
    p_cv_ref[2][0] = cv_comm.auto_exchange_pc2board_msg_.TransZ;
    Matrixf<3,3> rpy_cv_ref;
    //根据Rodrigues公式进行解算得到旋转矩阵
    rpy_cv_ref[0][0] = cosf(theta)+(1-cosf(theta))*Rotate_axis[0][0]*Rotate_axis[0][0];
    rpy_cv_ref[1][0] = (1-cosf(theta))*Rotate_axis[0][0]*Rotate_axis[1][0]+sinf(theta)*Rotate_axis[2][0];
    rpy_cv_ref[2][0] = (1-cosf(theta))*Rotate_axis[0][0]*Rotate_axis[2][0]-sinf(theta)*Rotate_axis[1][0];
    rpy_cv_ref[0][1] = (1-cosf(theta))*Rotate_axis[0][0]*Rotate_axis[1][0]-sinf(theta)*Rotate_axis[2][0];
    rpy_cv_ref[1][1] = cosf(theta)+(1-cosf(theta))*Rotate_axis[1][0]*Rotate_axis[1][0];
    rpy_cv_ref[2][1] = (1-cosf(theta))*Rotate_axis[1][0]*Rotate_axis[2][0]+sinf(theta)*Rotate_axis[0][0];
    rpy_cv_ref[0][2] = (1-cosf(theta))*Rotate_axis[0][0]*Rotate_axis[2][0]+sinf(theta)*Rotate_axis[1][0];
    rpy_cv_ref[1][2] = (1-cosf(theta))*Rotate_axis[1][0]*Rotate_axis[2][0]-sinf(theta)*Rotate_axis[0][0];
    rpy_cv_ref[2][2] = cosf(theta)+(1-cosf(theta))*Rotate_axis[2][0]*Rotate_axis[2][0];
    cv_t = robotics::rp2t( rpy_cv_ref*R0_, p_cv_ref);

    Matrixf<3, 1> rpy_ref;
    Matrixf<3,1> p_ref;

    rpy_ref = robotics::t2rpy(cv_t);
    p_ref = robotics::t2p(cv_t);

    this->ref_.x = p_ref[0][0];
    this->ref_.y = p_ref[1][0];
    this->ref_.z = p_ref[2][0];
    this->ref_.yaw = rpy_ref[0][0];
    this->ref_.pitch = rpy_ref[1][0];
    this->ref_.roll = rpy_ref[2][0];

    test2 = this->ref_.x;

    return cv_t;

}


void AutoExchangeController::auto_follow()
{

    cv2t();

    arm.ref_.x = this->ref_.x;
    arm.ref_.y = this->ref_.y;
    arm.ref_.z = this->ref_.z;

    arm.ref_.yaw = this->ref_.yaw;
    arm.ref_.pitch = this->ref_.pitch;
    arm.ref_.roll = this->ref_.roll;
}