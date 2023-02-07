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
#include <memory>

#ifndef INCLUDE_HOBOT_CODEC_DATA_H_
#define INCLUDE_HOBOT_CODEC_DATA_H_

// codec类型
enum class HobotCodecType {
  INVALID = 0,
  // 编码
  ENCODER,
  // 解码
  DECODER,
};

// codec状态
enum class CodecStatType {
  STOP = 0,
  START
};

// 数据类型
enum class CodecImgFormat {
  FORMAT_INVALID = 0,
  FORMAT_NV12,
  FORMAT_H264,
  FORMAT_H265,
  FORMAT_JPEG,
  FORMAT_MJPEG,
  FORMAT_BGR,
};

// 数据帧信息
struct FrameInfo {
  FrameInfo(uint64_t img_idx, struct timespec img_ts, struct timespec img_recved_ts) :
    img_idx_(img_idx), img_ts_(img_ts), img_recved_ts_(img_recved_ts)
  {}

  // 编号
  uint64_t img_idx_;
  // 图片时间戳
  struct timespec img_ts_;
  // 订阅到图片时刻的时间戳
  struct timespec img_recved_ts_;
  // codec处理完成时刻的时间戳
  struct timespec img_processed_ts_;
};

// 输出数据
struct OutputFrameDataType {
  // 编码输出地址
  uint8_t *mPtrData = nullptr;
  // 解码输出地址
  uint8_t *mPtrY;
  uint8_t *mPtrUV;
  // 分辨率
  int mWidth;
  int mHeight;
  // 大小
  int mDataLen;
  // 类型
  CodecImgFormat mFrameFmt;
  // 其他信息
  std::shared_ptr<FrameInfo> sp_frame_info = nullptr;
};

// codec初始化参数
// 不同平台codec可基于此类型进行扩展
struct HobotCodecParaBase {
  HobotCodecParaBase() {}
  virtual ~HobotCodecParaBase() {}

  std::string in_format_ = "rgb8";
  std::string out_format_ = "jpeg";
  HobotCodecType hobot_codec_type = HobotCodecType::INVALID;
  int framerate_ = 30;
  int mChannel_ = 0;
  float enc_qp_ = 10.0;
  float jpg_quality_ = 10.0;
};

#endif  // INCLUDE_HOBOT_CODEC_DATA_H_
