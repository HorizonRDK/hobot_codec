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

#include "rclcpp/rclcpp.hpp"
#include "hobot_venc.h"
#include "decoder/hobot_decoder_common.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/legacy/constants_c.h"

HobotVenc::~HobotVenc()
{
  if (m_DataTmp) {
    delete[] m_DataTmp;
    m_DataTmp = nullptr;
  }
  if (m_transformData.mPtrData) {
    delete[] m_transformData.mPtrData;
    m_transformData.mPtrData = nullptr;
  }
}

int HobotVenc::Init(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para) {
  if (!sp_hobot_codec_para || 0 != CheckParams(sp_hobot_codec_para)) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "Invalid codec para");
    return -1;
  }

  if (strcmp(sp_hobot_codec_para->out_format_.data(), "h264") == 0)
    m_enPalType = CodecImgFormat::FORMAT_H264;
  else if (strcmp(sp_hobot_codec_para->out_format_.data(), "h265") == 0)
    m_enPalType = CodecImgFormat::FORMAT_H265;
  else if (strcmp(sp_hobot_codec_para->out_format_.data(), "jpeg") == 0 ||
            strcmp(sp_hobot_codec_para->out_format_.data(), "jpeg-compressed") == 0)
    m_enPalType = CodecImgFormat::FORMAT_JPEG;
  else {
    RCLCPP_INFO(rclcpp::get_logger("HobotVenc"), "Invalid in_format: %s",
    sp_hobot_codec_para->in_format_.c_str());
    return -1;
  }


  m_fJpgQuality = sp_hobot_codec_para->jpg_quality_;
  m_fEncQp = sp_hobot_codec_para->enc_qp_;

  return 0;
}

int HobotVenc::DeInit() {
  int ret = Stop();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "Stop fail! ret: %d", ret);
    return ret;
  }
  
  return 0;
}

int HobotVenc::Stop() {
  if (CodecStatType::STOP != codec_stat_)
      codec_stat_ = CodecStatType::STOP;
  return 0;
}


int HobotVenc::Start(int nPicWidth, int nPicHeight) {
  if (-1 == nPicWidth || -1 == nPicHeight)
      return 0;

  if (CodecStatType::START == codec_stat_) {
    if (init_pic_w_ != nPicWidth || init_pic_h_ != nPicHeight) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"),
                    "Received image size has changed! "
                    " received image width: %d height: %d, the original width: %d height: %d",
                    nPicWidth,
                    nPicHeight,
                    init_pic_w_,
                    init_pic_h_);
      return -1;
    }
    return 0;
  }
  init_pic_w_ = nPicWidth;
  init_pic_h_ = nPicHeight;
  codec_stat_ = CodecStatType::START;
  
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "Start success");
  return 0;
}

// 如果 h26x 数据，则有 头部处理逻辑
int HobotVenc::Input(const uint8_t *pDataIn, int nPicWidth, int nPicHeight, int nLen, const struct timespec &time_stamp) {
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "Input data w: %d, h: %d, len: %d", nPicWidth, nPicHeight, nLen);

  int ret = Start(nPicWidth, nPicHeight);
  if (ret != 0 ) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"),
      "Start codec failed!");
    return ret;
  }

  // 校验解码器状态
  if (CodecStatType::START != codec_stat_) {
		RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"),
		"Codec is not start! codec_stat_: %d", static_cast<int>(codec_stat_));
		return -1;
  }
  if(m_status == false) {
    if (m_enPalType == CodecImgFormat::FORMAT_JPEG || m_enPalType == CodecImgFormat::FORMAT_MJPEG) {
      cv::Mat src(nPicHeight , nPicWidth, CV_8UC3, (void*)pDataIn);
      std::vector<uint8_t> buff;
      std::vector<int> quality;
      quality.push_back(cv::IMWRITE_JPEG_QUALITY);
      quality.push_back((int)m_fJpgQuality); 
      cv::imencode(".jpg", src, buff, quality);
      
      if (nullptr == m_transformData.mPtrData) {
        m_MtxFrame.lock();
        m_transformData.mPtrData = new uint8_t[buff.size()];
        m_transformData.mWidth = nPicWidth;
        m_transformData.mHeight = nPicHeight;
        m_transformData.mDataLen = buff.size();
        m_transformData.mFrameFmt = m_enPalType;
        m_MtxFrame.unlock();
      } 

      m_MtxFrame.lock();
      memcpy(m_transformData.mPtrData, reinterpret_cast<void*>(&buff[0]), buff.size());
      m_status = true;
      m_MtxFrame.unlock();
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"),"Only encoding of JPEG is supported");
      rclcpp::shutdown();
    }
    
	}
  return 0;
}

int HobotVenc::ReleaseOutput(const std::shared_ptr<OutputFrameDataType>& pFrame)
{
  if (pFrame) {
      RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "[%s] y: 0x%x, uv: 0x%x, w: %d, h: %d",
          __func__,
          pFrame->mPtrY, pFrame->mPtrUV, 
          pFrame->mWidth, pFrame->mHeight);
  }
  return 0;
}

int HobotVenc::GetOutput(std::shared_ptr<OutputFrameDataType> pOutFrm) {
  if (CodecStatType::START == codec_stat_ && nullptr != m_transformData.mPtrData && m_status == true) {
    m_MtxFrame.lock();
    if(m_DataTmp == nullptr) {
      m_DataTmp = new uint8_t[m_transformData.mDataLen];
    }
    memcpy(m_DataTmp, m_transformData.mPtrData, m_transformData.mDataLen);
    pOutFrm->mPtrData = m_DataTmp;
    pOutFrm->mWidth = m_transformData.mWidth;
    pOutFrm->mHeight = m_transformData.mHeight;
    pOutFrm->mDataLen = m_transformData.mDataLen;
    pOutFrm->mFrameFmt = m_transformData.mFrameFmt;
    m_status = false;
    m_MtxFrame.unlock();
    return 0;
  }
  return -1;
}

int HobotVenc::CheckParams(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para)
{
  if (!sp_hobot_codec_para) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "Invalid input");
    return -1;
  }

  return 0;
}

