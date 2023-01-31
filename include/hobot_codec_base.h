// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <list>
#include <thread>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "include/hobot_codec_data.h"

#ifndef INCLUDE_HOBOT_CODEC_BASE_H_
#define INCLUDE_HOBOT_CODEC_BASE_H_

// 缓存数量
#define BUF_CNT 5

class HobotCodecBase {
 public:
  HobotCodecBase() {}
  virtual ~HobotCodecBase() {}

  virtual int Init(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para) = 0;
  virtual int DeInit() = 0;
  virtual int Input(const uint8_t *pDataIn, int nPicWidth, int nPicHeight, int nLen,
                    const struct timespec &time_stamp) = 0;
  virtual int GetOutput(std::shared_ptr<OutputFrameDataType> pFrame) = 0;
  virtual int ReleaseOutput(const std::shared_ptr<OutputFrameDataType>& pFrame) = 0;

 protected:
  // 启动codec
  // 通过Input接口向codec输入数据时，调用Start接口启动codec
  // 如果未启动过，使用输入数据分辨率参数启动codec
  // 如果已经启动过，校验输入数据分辨率参数和启动时分辨率是否一致
  // Input接口中调用
  virtual int Start(int nPicWidth, int nPicHeight) = 0;

  // 停止codec
  // DeInit接口中调用
  virtual int Stop() = 0;

  // codec配置参数校验，编码、解码以及不同平台的校验方法不同，子类中可继承实现校验
  // Init时调用
  virtual int CheckParams(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para);
  
  // 启动codec时的分辨率，Start接口中更新
  int init_pic_w_;
  int init_pic_h_;
  
  // 初始化时申请的缓存大小
  int codec_buf_size_ = 0;
  
  // 通道号
  int codec_chn_ = -1;

  // codec状态
  CodecStatType codec_stat_ = CodecStatType::STOP;
};


#endif  // INCLUDE_HOBOT_CODEC_BASE_H_
