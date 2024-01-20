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

#include "base/cv_comm/cv_comm.h"
#include "cmsis_os.h"

CVComm::CVComm(UART_HandleTypeDef* huart)
    : huart_(huart),
      unpack_step_(WAIT),
      general_connect_(200),
      aim_shoot_connect_(25),
      navigation_connect_(200),
      game_status_connect_(1000),
      auto_exchange_connect_(1000){}

// Init UART receive 初始化，打开UART接收
void CVComm::init(void) {
  if (huart_ != nullptr) {
    HAL_UART_Receive_IT(huart_, rx_.byte, 1);
  }
}

// Handle mode, check connection 处理通信模式，检查连接状态
void CVComm::handle(void) {
  general_connect_.check();
  aim_shoot_connect_.check();
  navigation_connect_.check();
  game_status_connect_.check();
  auto_exchange_connect_.check();

  general_board2pc_msg_.mode = (uint8_t)mode_;
}

// Data transmit monitor 数据发送管理
void CVComm::txMonitor(uint32_t* tick) {
  cvcomm::MsgStream_e stream = cvcomm::MsgStream::BOARD2PC;

  txMsg(stream, cvcomm::MsgType::GENERAL);
  osDelayUntil(tick, 5);

  // 自瞄/打符/反符
  if (mode_ == CVMode::AUTOAIM || mode_ == CVMode::ENERGY ||
      mode_ == CVMode::ENERGY_DISTURB) {
    for (int i = 0; i < 20; i++) {
      txMsg(stream, cvcomm::MsgType::AIM_SHOOT);
      osDelayUntil(tick, 5);
    }
  }
  // 导航
  else if (mode_ == CVMode::NAVIGATION) {
    for (int i = 0; i < 20; i++) {
      txMsg(stream, cvcomm::MsgType::NAVIGATION);
      osDelayUntil(tick, 5);
    }
  }
  // 决策（导航+自瞄）
  else if (mode_ == CVMode::DECISION) {
    for (int i = 0; i < 10; i++) {
      txMsg(stream, cvcomm::MsgType::AIM_SHOOT);
      osDelayUntil(tick, 5);
      txMsg(stream, cvcomm::MsgType::NAVIGATION);
      osDelayUntil(tick, 5);
    }
    txMsg(stream, cvcomm::MsgType::GAME_STATUS);
    osDelayUntil(tick, 10);
  } else {
    osDelayUntil(tick, 95);
  }
}

// Data transmit 数据发送
void CVComm::txMsg(cvcomm::MsgStream_e msg_stream, cvcomm::MsgType_e msg_type) {
  // start of frame(0x23)
  tx_.frame.sof = CV_COMM_SOF;
  memcpy(tx_.buf, &tx_.frame.sof, sizeof(tx_.frame.sof));
  tx_.pack_size = sizeof(tx_.frame.sof);

  // pack id
  tx_.frame.pack_id = (uint8_t)msg_stream + (uint8_t)msg_type;
  memcpy(tx_.buf + tx_.pack_size, &tx_.frame.pack_id,
         sizeof(tx_.frame.pack_id));
  tx_.pack_size += sizeof(tx_.frame.pack_id);

  // data
  if (msg_stream == cvcomm::MsgStream::PC2BOARD) {
    if (msg_type == cvcomm::MsgType::GENERAL) {
      appendTxData(general_pc2board_msg_);
    } else if (msg_type == cvcomm::MsgType::AIM_SHOOT) {
      appendTxData(aim_shoot_pc2board_msg_);
    } else if (msg_type == cvcomm::MsgType::NAVIGATION) {
      appendTxData(navigation_pc2board_msg_);
    } else if (msg_type == cvcomm::MsgType::GAME_STATUS) {
      appendTxData(game_status_pc2board_msg_);
    }
  } else if (msg_stream == cvcomm::MsgStream::BOARD2PC) {
    if (msg_type == cvcomm::MsgType::GENERAL) {
      appendTxData(general_board2pc_msg_);
    } else if (msg_type == cvcomm::MsgType::AIM_SHOOT) {
      appendTxData(aim_shoot_board2pc_msg_);
    } else if (msg_type == cvcomm::MsgType::NAVIGATION) {
      appendTxData(navigation_board2pc_msg_);
    } else if (msg_type == cvcomm::MsgType::GAME_STATUS) {
      appendTxData(game_status_board2pc_msg_);
    }
  }

  // tail(crc16)
  tx_.pack_size += sizeof(tx_.frame.crc16);
  CRC16_Append(tx_.buf, tx_.pack_size);

  // UART transmit
  if (huart_ != nullptr) {
    HAL_UART_Transmit_IT(huart_, tx_.buf, tx_.pack_size);
  }
}

// Data receive callback 接收中断
void CVComm::rxCallback(void) {
  // 将接收到的1byte数据存入fifo缓冲区
  rx_.fifo.append(rx_.byte, 1);

  // 数据解包
  // 等待阶段
  if (unpack_step_ == WAIT) {
    uint32_t sof_index = rx_.fifo.find(CV_COMM_SOF);
    if (sof_index != rx_.fifo.size()) {
      // 检测到SOF标志，清除缓冲区无效数据，进入id阶段
      rx_.fifo.remove(sof_index);
      rx_.expect_size = sizeof(rx_.frame.sof) + sizeof(rx_.frame.pack_id);
      unpack_step_ = PACK_ID;
    } else {
      // 未检测到SOF标志，清空缓冲区
      rx_.fifo.clear();
    }
  }
  // id阶段
  if (unpack_step_ == PACK_ID) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      // 进入data_len阶段
      memcpy(&rx_.frame.pack_id, rx_.buf + sizeof(rx_.frame.sof),
             sizeof(rx_.frame.pack_id));
      rx_.expect_size += sizeof(rx_.frame.data_len);
      unpack_step_ = DATA_LEN;
    }
  }
  // data_len阶段，设置data段需接收的数据长度
  if (unpack_step_ == DATA_LEN) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      // 进入data阶段
      memcpy(&rx_.frame.data_len,
             rx_.buf + sizeof(rx_.frame.sof) + sizeof(rx_.frame.pack_id),
             sizeof(rx_.frame.data_len));
      rx_.expect_size += rx_.frame.data_len;
      if (rx_.expect_size > rx_.fifo.maxSize()) {
        // 数据长度错误
        rx_.fifo.clear();
        unpack_error_ = DATA_LEN_OUTRANGE;
        unpack_step_ = WAIT;
      } else {
        unpack_step_ = DATA;
      }
    }
  }
  // data阶段，接收数据量足够后进入帧尾校验阶段
  if (unpack_step_ == DATA) {
    if (rx_.fifo.size() >= rx_.expect_size) {
      // 进入tail阶段
      rx_.expect_size += sizeof(rx_.frame.crc16);
      unpack_step_ = CRC16;
      unpack_step_ = READ_DATA;
    }
  }//todo:解决crc错误问题
  // 帧尾CRC16校验阶段，接收完帧尾后进行校验
//  if (unpack_step_ == CRC16) {
//    if (rx_.fifo.size() >= rx_.expect_size) {
//      rx_.frame.crc16 = CRC16_Calc(rx_.buf, rx_.expect_size);
//      if (CRC16_Verify(rx_.buf, rx_.expect_size)) {
//        // CRC校验成功
//        unpack_step_ = READ_DATA;
//      } else {
//        // CRC校验失败
//        rx_.fifo.clear();
//        unpack_error_ = CRC_FAIL;
//        unpack_step_ = WAIT;
//      }
//    }
//  }
  // read_data阶段，从缓冲区读取数据
  if (unpack_step_ == READ_DATA) {
    const static uint8_t offset = sizeof(rx_.frame.sof) +
                                  sizeof(rx_.frame.pack_id) +
                                  sizeof(rx_.frame.data_len);
    if ((rx_.frame.pack_id & 0xf0) == (uint8_t)cvcomm::MsgStream::PC2BOARD) {
      unpack_error_ = NO_ERROR;
      //判断信息类型
      if ((rx_.frame.pack_id & 0x0f) == (uint8_t)cvcomm::MsgType::GENERAL) {
        memcpy(&general_pc2board_msg_, rx_.buf + offset, rx_.frame.data_len);
        general_connect_.refresh();
      } else if ((rx_.frame.pack_id & 0x0f) ==
                 (uint8_t)cvcomm::MsgType::AIM_SHOOT) {
        memcpy(&aim_shoot_pc2board_msg_, rx_.buf + offset, rx_.frame.data_len);
        aim_shoot_connect_.refresh();
      } else if ((rx_.frame.pack_id & 0x0f) ==
                 (uint8_t)cvcomm::MsgType::NAVIGATION) {
        memcpy(&navigation_pc2board_msg_, rx_.buf + offset, rx_.frame.data_len);
        navigation_connect_.refresh();
      } else if ((rx_.frame.pack_id & 0x0f) ==
                 (uint8_t)cvcomm::MsgType::GAME_STATUS) {
        memcpy(&game_status_pc2board_msg_, rx_.buf + offset,
               rx_.frame.data_len);
        game_status_connect_.refresh();
      }else if ((rx_.frame.pack_id & 0x0f) ==
                 (uint8_t)cvcomm::MsgType::AUTO_EXCHANGE) {
          memcpy(&auto_exchange_pc2board_msg_, rx_.buf + offset,
                 rx_.frame.data_len);
          //解包
          uint8_t tmp[sizeof(auto_exchange_pc2board_msg_)];
          memcpy(tmp,&auto_exchange_pc2board_msg_,rx_.frame.data_len);
          auto_exchange_pc2board_msg_.TransX = (int16_t)( tmp[0]|tmp[1]<<8 )/ 10.f;
          auto_exchange_pc2board_msg_.TransY = (int16_t)( tmp[2]|tmp[3]<<8 )/ 10.f;
          auto_exchange_pc2board_msg_.TransZ = (int16_t)( tmp[4]|tmp[5]<<8 )/ 10.f;
          auto_exchange_pc2board_msg_.RotX = (int16_t)( tmp[6]|tmp[7]<<8 )/ 10000.f;
          auto_exchange_pc2board_msg_.RotY = (int16_t)( tmp[8]|tmp[9]<<8 )/ 10000.f;
          auto_exchange_pc2board_msg_.RotZ = (int16_t)( tmp[10]|tmp[11]<<8 )/ 10000.f;
          //解包完成
          auto_exchange_connect_.refresh();
      } else {
        unpack_error_ = ID_UNDEFINED;  // 未定义id
      }
    } else if ((rx_.frame.pack_id & 0xf0) ==
               (uint8_t)cvcomm::MsgStream::BOARD2PC) {
      unpack_error_ = NO_ERROR;
      if ((rx_.frame.pack_id & 0x0f) == (uint8_t)cvcomm::MsgType::GENERAL) {
        memcpy(&general_board2pc_msg_, rx_.buf + offset, rx_.frame.data_len);
        general_connect_.refresh();
      } else if ((rx_.frame.pack_id & 0x0f) ==
                 (uint8_t)cvcomm::MsgType::AIM_SHOOT) {
        memcpy(&aim_shoot_board2pc_msg_, rx_.buf + offset, rx_.frame.data_len);
        aim_shoot_connect_.refresh();
      } else if ((rx_.frame.pack_id & 0x0f) ==
                 (uint8_t)cvcomm::MsgType::NAVIGATION) {
        memcpy(&navigation_board2pc_msg_, rx_.buf + offset, rx_.frame.data_len);
        navigation_connect_.refresh();
      } else if ((rx_.frame.pack_id & 0x0f) ==
                 (uint8_t)cvcomm::MsgType::GAME_STATUS) {
        memcpy(&game_status_board2pc_msg_, rx_.buf + offset,
               rx_.frame.data_len);
        game_status_connect_.refresh();
      } else {
        unpack_error_ = ID_UNDEFINED;  // 未定义id
      }
    }
    rx_.fifo.remove(rx_.expect_size);  // 从缓冲区移除该帧数据
    unpack_step_ = WAIT;               // 重置解包状态
  }

  // 重新打开串口接收
  if (huart_ != nullptr) {
    HAL_UART_Receive_IT(huart_, rx_.byte, 1);
  }
}
