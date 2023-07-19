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

#include <sys/times.h>
#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "include/video_utils.hpp"
#include "include/hobot_codec_impl.h"
#include "include/hobot_codec_base.h"
#include "hobot_vdec.h"
#include "hobot_venc.h"

#define PUB_QUEUE_NUM 5

HobotCodecImpl::HobotCodecImpl() {
  std::string platform;
#ifdef PLATFORM_X3
  platform = "platform x3";
#else
  #ifdef PLATFORM_Rdkultra
    platform = "platform rdkultra";
  #else
    # ifdef PLATFORM_X86
      platform = "platform x86";
    #endif
  #endif
#endif
  if (platform.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("HobotCodecImpl"), "Unknown platform!");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("HobotCodecImpl"), "%s", platform.data());
  }
}

HobotCodecImpl::~HobotCodecImpl() {
}

int HobotCodecImpl::Init(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para) {
  // 收到数据才可以知道 宽高，才能真正创建
  // 此处只创建实例，订阅到消息后根据具体分辨率进行codec的初始化
  if (HobotCodecType::ENCODER == sp_hobot_codec_para->hobot_codec_type) {
    RCLCPP_INFO(rclcpp::get_logger("HobotVdecImpl"), "Creat HobotVenc");
    // 输出是压缩格式，创建编码器
    sp_codec_base_ = std::make_shared<HobotVenc>();
  } else {
    RCLCPP_INFO(rclcpp::get_logger("HobotVdecImpl"), "Creat HobotVdec");
    // 输出是非压缩格式，创建解码器
    sp_codec_base_ = std::make_shared<HobotVdec>();
  }

  if (!sp_codec_base_) {
    return -1;
  }

  return sp_codec_base_->Init(sp_hobot_codec_para);
}

int HobotCodecImpl::DeInit() {
  std::unique_lock<std::mutex> lock(frame_mtx_);
  frame_cv_.notify_one();
  lock.unlock();

  if (sp_codec_base_) {
    int ret = sp_codec_base_->DeInit();
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotCodecImpl"), "DeInit fail! ret: %d", ret);
      return ret;
    }
  }

  return 0;
}

int HobotCodecImpl::Input(const uint8_t *pDataIn, int nPicWidth, int nPicHeight, int nLen, std::shared_ptr<FrameInfo> frame_info) {
  if (!pDataIn) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecImpl"), "Invalid input data");
    return -1;
  }
  if (!frame_info) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecImpl"), "Invalid frame_info");
    return -1;
  }

  if (!sp_codec_base_) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecImpl"), "Hobot codec impl is invalid");
    return -1;
  }

  int ret = sp_codec_base_->Input(pDataIn, nPicWidth, nPicHeight, nLen, frame_info->img_ts_);
  if (0 != ret) {
    RCLCPP_WARN(rclcpp::get_logger("HobotCodecImpl"), "Input fail, ret: %d", ret);
  } else {
    std::unique_lock<std::mutex> lock(frame_mtx_);
    frame_list_.push_back(frame_info);
    frame_cv_.notify_one();
    lock.unlock();
  }

  return ret;
}

std::shared_ptr<OutputFrameDataType> HobotCodecImpl::GetOutput() {
  if (!sp_codec_base_) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecImpl"), "Hobot codec impl is invalid");
    return nullptr;
  }
  std::shared_ptr<OutputFrameDataType> pFrame = nullptr;

  std::unique_lock<std::mutex> lock(frame_mtx_);
  frame_cv_.wait(lock, [this](){
    return !frame_list_.empty() || !rclcpp::ok();
  });
  lock.unlock();

  if (frame_list_.empty() || !rclcpp::ok()) {
    return pFrame;
  }
  
  pFrame = std::make_shared<OutputFrameDataType>();
  pFrame->sp_frame_info = frame_list_.front();
  frame_list_.pop_front();
  frame_mtx_.unlock();
  if (0 != sp_codec_base_->GetOutput(pFrame)) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecImpl"), "GetOutput fail");
    return nullptr;
  }
  struct timespec time_now;
  clock_gettime(CLOCK_REALTIME, &time_now);
  pFrame->sp_frame_info->img_processed_ts_ = time_now;
  return pFrame;
}

int HobotCodecImpl::ReleaseOutput(const std::shared_ptr<OutputFrameDataType>& pFrame) {
  return sp_codec_base_->ReleaseOutput(pFrame);
}
