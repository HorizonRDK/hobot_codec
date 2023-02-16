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
#include <thread>
#include <memory>

#include "include/hobot_codec_data.h"

#ifndef INCLUDE_HOBOT_CODEC_IMPL_H_
#define INCLUDE_HOBOT_CODEC_IMPL_H_

class HobotCodecBase;

class HobotCodecImpl {
 public:
  HobotCodecImpl();
  ~HobotCodecImpl();

  // 初始化
  // - 参数
  //   - [in] sp_hobot_codec_para 初始化参数。
  // - 返回值 0：成功，-1：失败
  int Init(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para);

  // 反初始化
  // - 返回值 0：成功，-1：失败
  int DeInit();

  // codec输入，数据送给codec后就返回，通过GetOutput接口异步获取输出
  // - 参数
  //   - [in] pDataIn 输入数据地址
  //   - [in] nPicWidth 输入数据宽度
  //   - [in] nPicHeight 输入数据高度
  //   - [in] nLen 输入数据大小
  //   - [in] frame_info 输入数据信息
  // - 返回值 0：成功，-1：失败
  int Input(const uint8_t *pDataIn, int nPicWidth, int nPicHeight, int nLen, std::shared_ptr<FrameInfo> frame_info);

  // 获取codec输出
  // - 返回值 成功：codec输出数据，失败：nullptr
  std::shared_ptr<OutputFrameDataType> GetOutput();

  // 释放codec输出
  // - 参数
  //   - [in] pFrame codec输出数据
  // - 返回值 0：成功，-1：失败
  int ReleaseOutput(const std::shared_ptr<OutputFrameDataType>& pFrame);

 private:
  // 时间戳队列，用于codec输出与输入的时间戳对应
  std::mutex frame_mtx_;
  std::condition_variable frame_cv_;
  std::list<std::shared_ptr<FrameInfo>> frame_list_;

  // codec抽象类对象，编码、解码、不同平台使用同一个抽象类
  std::shared_ptr<HobotCodecBase> sp_codec_base_ = nullptr;
};


#endif  // INCLUDE_HOBOT_CODEC_IMPL_H_
