/**
 ******************************************************************************
 * @file    cv_comm.cpp/h
 * @brief   CV-control communication(UART). 视觉-电控通信(UART)
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

#ifndef CV_COMM_H
#define CV_COMM_H

#include "base/common/connect.h"
#include "base/common/crc.h"
#include "base/common/fifo_buffer.h"
#include "base/cv_comm/cv_protocol.h"
#include "usart.h"

#define CV_COMM_TX_BUF_SIZE CV_COMM_MAX_SIZE
#define CV_COMM_RX_BUF_SIZE CV_COMM_MAX_SIZE

// CV-control communication class
// 电控视觉通信类
class CVComm {
 public:
  CVComm(UART_HandleTypeDef* huart = nullptr);

  // Init UART receive 初始化，打开UART接收
  void init(void);
  // Handle mode, check connection 处理通信模式，检查连接状态
  void handle(void);
  // Data transmit monitor 数据发送管理
  void txMonitor(uint32_t* tick);

  // Data transmit 数据发送
  void txMsg(cvcomm::MsgStream_e msg_stream, cvcomm::MsgType_e msg_type);
  // Data receive callback 接收中断
  void rxCallback(void);
  // UART端口校验
  bool uartCheck(UART_HandleTypeDef* huart) { return huart == huart_; }

 private:
  // Append data length & data to tx buffer
  // 添加数据段长度和数据到发送缓冲区
  template <typename T>
  void appendTxData(const T& data) {
    tx_.frame.data_len = sizeof(data);
    memcpy(tx_.buf + tx_.pack_size, &tx_.frame.data_len,
           sizeof(tx_.frame.data_len));
    tx_.pack_size += sizeof(tx_.frame.data_len);
    memcpy(tx_.buf + tx_.pack_size, &data, sizeof(data));
    tx_.pack_size += sizeof(data);
  }

 public:
  CVMode mode_;

  Connect general_connect_;
  cvcomm::general::PC2Board_t general_pc2board_msg_;
  cvcomm::general::Board2PC_t general_board2pc_msg_;

  Connect aim_shoot_connect_;
  cvcomm::aimshoot::PC2Board_t aim_shoot_pc2board_msg_;
  cvcomm::aimshoot::Board2PC_t aim_shoot_board2pc_msg_;

  Connect navigation_connect_;
  cvcomm::navigation::PC2Board_t navigation_pc2board_msg_;
  cvcomm::navigation::Board2PC_t navigation_board2pc_msg_;

  Connect game_status_connect_;
  cvcomm::gamestatus::PC2Board_t game_status_pc2board_msg_;
  cvcomm::navigation::Board2PC_t game_status_board2pc_msg_;

  Connect auto_exchange_connect_;
  cvcomm::autoexchange::PC2Board_t auto_exchange_pc2board_msg_;
  cvcomm::autoexchange::Board2PC_t auto_exchange_board2pc_msg_;

 private:
  UART_HandleTypeDef* huart_;

  struct Tx_t {
    uint8_t buf[CV_COMM_TX_BUF_SIZE];
    cvcomm::Frame_t frame;
    uint32_t pack_size;
  } tx_;

  struct Rx_t {
    Rx_t(void) : fifo(buf, CV_COMM_RX_BUF_SIZE) {}

    uint8_t byte[1];
    uint8_t buf[CV_COMM_RX_BUF_SIZE];
    FIFOBuffer fifo;
    cvcomm::Frame_t frame;
    uint32_t expect_size;
  } rx_;

  enum MessageUnpackStep_e {
    WAIT,
    PACK_ID,
    DATA_LEN,
    DATA,
    CRC16,
    READ_DATA,
  } unpack_step_;

  enum UnpackError_e {
    NO_ERROR,
    ID_UNDEFINED,
    DATA_LEN_OUTRANGE,
    CRC_FAIL,
  } unpack_error_;
};

#endif  // CV_COMM_H