/**
 ******************************************************************************
 * @file    cv_protocol.h
 * @brief   CV-control communication protocol. 视觉-电控通信协议
 * @author  Spoon Guan
 ******************************************************************************
 * @protocol 通信协议
 * 1. UART config 串口配置
 *    波特率115200，8位数据位，1位停止位，无硬件流控，无校验位
 * 2. frame format 数据帧格式
 *    header(3-byte)+data(n-byte)+tail(2-byte)
 *    header: SOF(1-byte, 0x23)+pack_id(1-byte)+data_len(1-byte,=n)
 *    data: n-byte
 *    tail: CRC16(2-byte)
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef CV_PROTOCOL_H
#define CV_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#ifndef __packed
#define __packed __attribute__((packed))
#endif

// CV工作模式
enum class CVMode : uint8_t {
  // mode: IDLE
  // CV工作线程: io
  // 通信包
  // CV->board: GENERAL
  // board->CV: GENERAL
  IDLE = 0x00,

  // mode: AUTOAIM(自瞄)
  // CV工作线程: io, autoaim
  // 通信包
  // CV->board: GENERAL, AIM_SHOOT
  // board->CV: GENERAL, AIM_SHOOT
  AUTOAIM = 0x01,

  // mode: ENERGY(能量机关)
  // CV工作线程: io, energy
  // 通信包
  // CV->board: GENERAL, AIM_SHOOT
  // board->CV: GENERAL, AIM_SHOOT
  ENERGY = 0x02,

  // mode: ENERGY_DISTURB(反符)
  // CV工作线程: io, energy
  // 通信包
  // CV->board: GENERAL, AIM_SHOOT
  // board->CV: GENERAL, AIM_SHOOT
  ENERGY_DISTURB = 0x03,

  // mode: NAVIGATION(导航)
  // CV工作线程: io, navigation
  // 通信包
  // CV->board: GENERAL, NAVIGATION
  // board->CV: GENERAL, NAVIGATION
  NAVIGATION = 0x04,

  // mode: DECISION(决策)
  // CV工作线程: io, autoaim, navigation, decision
  // 通信包
  // CV->board: GENERAL, AUTOAIM, NAVIGATION
  // board->CV: GENERAL, AUTOAIM, NAVIGATION, GAME_STATUS
  DECISION = 0x05,

  AUTO_EXCHANGE = 0x06,
  // todo: 工程相关注释
};

#define CV_COMM_MAX_SIZE 64
#define CV_COMM_SOF 0x23  // start of frame

namespace cvcomm {

// Data frame
typedef struct Frame {
  uint8_t sof;
  uint8_t pack_id;
  uint8_t data_len;
  // uint8_t data[n];
  uint16_t crc16;
} Frame_t;

// Pack ID

// 数据传输方向
typedef enum MsgStream {
  PC2BOARD = 0x00,
  BOARD2PC = 0x10,
} MsgStream_e;

// 数据包类别
typedef enum MsgType {
  GENERAL = 0x00,      // 通用(模式切换)数据包
  AIM_SHOOT = 0x01,    // 云台+发射数据包
  NAVIGATION = 0x02,   // 导航数据包
  GAME_STATUS = 0x03,  // 比赛信息数据包(用于决策)
  AUTO_EXCHANGE = 0x04,//自动兑矿数据包

  // todo: 工程相关(机械臂等)
} MsgType_e;

// general通用数据包----------------------------------------------------------------
namespace general {
// 通用数据包(视觉->电控) id-0x00
typedef struct PC2Board {
  uint8_t mode_fdb;  // 视觉工作模式反馈
} __packed PC2Board_t;

// 通用数据包(电控->视觉) id-0x10
typedef struct Board2PC {
  uint8_t mode;  // 视觉工作模式
} __packed Board2PC_t;
}  // namespace general

// aimshoot自瞄(瞄准发射相关)数据包--------------------------------------------------
namespace aimshoot {
// 自瞄数据包(视觉->电控) id-0x01
typedef struct PC2Board {
  // yaw左+右-，pitch下+上-(ZYX欧拉角，右手定则)
  float yaw_angle;     // deg
  float pitch_angle;   // deg
  float yaw_speed;     // dps
  float pitch_speed;   // dps
  float dist;          // m，视觉测量距离
  uint8_t shoot_flag;  // 发射标志
  uint32_t shoot_id;   // 发射id
  uint8_t enemy_id;    // 敌方id
} __packed PC2Board_t;

// 自瞄数据包(电控->视觉) id-0x11
typedef struct Board2PC {
  // yaw左+右-，pitch下+上-(ZYX欧拉角，右手定则)
  uint8_t self_id;        // 用于判断红蓝方
  float yaw_angle;        // deg
  float pitch_angle;      // deg
  float yaw_speed;        // dps
  float pitch_speed;      // dps
  float yaw_offset;       // deg，电控yaw补偿
  float pitch_offset;     // deg，电控pitch补偿
  float dist;             // m，电控测量距离
  float shoot_speed;      // m/s，射速
  uint8_t shoot_delay;    // ms 发射延迟
  uint32_t shoot_id_fdb;  // 发射id反馈(=对应的shoot_id)
  uint8_t is_big_energy;  // 0-小符，1-大符
  float chassis_vx;  // m/s，底盘前后速度，前+后-，用于预测和发射判断
  float chassis_vy;  // m/s，底盘左右速度，左+右-，用于预测和发射判断
} __packed Board2PC_t;
}  // namespace aimshoot

// navigation导航数据包-------------------------------------------------------
namespace navigation {
// 导航数据包(视觉->电控) id-0x02
typedef struct PC2Board {
  // 底盘速度指令
  float vx;  // m/s，底盘前后速度，前+后-
  float vy;  // m/s，底盘左右速度，左+右-
  float wz;  // dps，底盘旋转角速度，逆时针+顺时针-
} __packed PC2Board_t;

// 导航数据包(电控->视觉) id-0x12
typedef struct Board2PC {
  // 里程计(世界坐标系)
  float x;      // m
  float y;      // m
  float theta;  // deg
} __packed Board2PC_t;
}  // namespace navigation

// game Statusb比赛信息数据包--------------------------------------------------
namespace gamestatus {
// 比赛信息数据包(视觉->电控) id-0x03
typedef struct GameStatus_PC2Board {
  uint8_t reserve;
} __packed PC2Board_t;

// 比赛信息数据包(电控->视觉) id-0x13
typedef struct GameStatusMsg_Board2PC {
  uint8_t robot_id;
  uint8_t remain_HP;
  uint8_t max_HP;
  // 全体机器人血量
  struct GameRobotHP {
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
  } __packed game_robot_hp;
  // 场地事件数据
  // bit 0-2：
  // bit 0：己方补给站 1号补血点占领状态 1为已占领；
  // bit 1：己方补给站 2号补血点占领状态 1为已占领；
  // bit 2：己方补给站 3号补血点占领状态 1为已占领；
  // bit 3-5：己方能量机关状态：
  // • bit 3为打击点占领状态，1为占领；
  // • bit 4为小能量机关激活状态，1为已激活；
  // • bit 5为大能量机关激活状态，1为已激活；
  // bit 6：己方侧R2/B2环形高地占领状态1为已占领；
  // bit 7：己方侧R3/B3梯形高地占领状态 1为已占领；
  // bit 8：己方侧R4/B4梯形高地占领状态 1为已占领；
  // bit 9：己方基地护盾状态：
  // • 1为基地有虚拟护盾血量；
  // • 0为基地无虚拟护盾血量；
  // bit 10：己方前哨战状态：
  // • 1为前哨战存活；
  // • 0为前哨战被击毁；
  // bit 10-31: 保留
  uint32_t event_data;
  struct GameRobotPos {
    float x;
    float y;
    float z;
    float yaw;
  } __packed game_robot_pos;
} __packed Board2PC_t;
}  // namespace gamestatus

namespace autoexchange
{
    typedef struct AutoExchange_PC2Board {
    float TransX;
    float TransY;
    float TransZ;
    float RotX;
    float RotY;
    float RotZ;

    } __packed PC2Board_t;

    typedef struct AutoExchange_Board2PC {

    } __packed Board2PC_t;
    //todo:fill it
} //namespace autoexchange

}  // namespace cvcomm

#endif  // CV_PROTOCOL_H