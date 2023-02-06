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
#include "hobot_vdec.h"
#include "decoder/hobot_decoder_common.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>

HobotVdec::~HobotVdec()
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

int HobotVdec::Init(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para) {
  if (!sp_hobot_codec_para || 0 != CheckParams(sp_hobot_codec_para)) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "Invalid codec para");
    return -1;
  }

  if ("h264" == sp_hobot_codec_para->in_format_)
    m_enPalType = CodecImgFormat::FORMAT_H264;
  else if ("h265" == sp_hobot_codec_para->in_format_)
    m_enPalType = CodecImgFormat::FORMAT_H265;
  else if ("jpeg" == sp_hobot_codec_para->in_format_ ||
  "jpeg-compressed" == sp_hobot_codec_para->in_format_)
    m_enPalType = CodecImgFormat::FORMAT_JPEG;
  else {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "Invalid in_format: %s. %d",
    sp_hobot_codec_para->in_format_.c_str(), __LINE__);
    return -1;
  }

  return 0;
}

int HobotVdec::DeInit() {
  int ret = Stop();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "Stop fail! ret: %d", ret);
    return ret;
  }
  
  return 0;
}

int HobotVdec::Stop() {
  if (CodecStatType::STOP != codec_stat_)
      codec_stat_ = CodecStatType::STOP;
  return 0;
}


int HobotVdec::Start(int nPicWidth, int nPicHeight) {
  if (-1 == nPicWidth || -1 == nPicHeight)
      return 0;

  if (CodecStatType::START == codec_stat_) {
    // 已经初始化过
    // 不同分辨率直接失败不处理
    if (init_pic_w_ != nPicWidth || init_pic_h_ != nPicHeight) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"),
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

  // 收到第一帧，开始初始化
  init_pic_w_ = nPicWidth;
  init_pic_h_ = nPicHeight;
  codec_stat_ = CodecStatType::START;
  
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "Start success");
  return 0;
}

int HobotVdec::Input(const uint8_t *pDataIn, int nPicWidth, int nPicHeight, int nLen, const struct timespec &time_stamp) {
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "Input data w: %d, h: %d, len: %d", nPicWidth, nPicHeight, nLen);

  int ret = Start(nPicWidth, nPicHeight);
  if (ret != 0 ) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"),
      "Start codec failed!");
    return ret;
  }
  if (nLen > nPicWidth * nPicHeight * 3) {
      std::string fname = std::to_string(time_stamp.tv_sec) + "_" + std::to_string(time_stamp.tv_nsec) + ".jpeg";
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"),
      "input nLen: %d exceeds alloc size: %d, dump to file: %s",
      nLen, nPicWidth * nPicHeight * 3, fname.data());
      std::ofstream ofs(fname);
      ofs.write(reinterpret_cast<const char*>(pDataIn), nLen);
      return -1;
  }
  // 校验解码器状态
  if (CodecStatType::START != codec_stat_) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"),
      "Codec is not start! codec_stat_: %d", static_cast<int>(codec_stat_));
      return -1;
  }

	if(m_status == false) {
		if (nullptr == m_transformData.mPtrData) {
			m_MtxFrame.lock();
			m_transformData.mPtrData = new uint8_t[nPicWidth * nPicHeight * 3];
			m_transformData.mWidth = nPicWidth;
			m_transformData.mHeight = nPicHeight;
			m_transformData.mDataLen = nPicWidth * nPicHeight * 3;
			m_MtxFrame.unlock();
		} 
		if(m_enPalType == CodecImgFormat::FORMAT_JPEG) {
			std::vector<uint8_t> buff(nPicWidth * nPicHeight * 3);
			memcpy(reinterpret_cast<void*>(&buff[0]), pDataIn, nLen);
			cv::Mat out_data = cv::imdecode(buff, CV_LOAD_IMAGE_COLOR);
			m_MtxFrame.lock();
			memcpy(m_transformData.mPtrData, out_data.ptr<uint8_t>(), nPicWidth * nPicHeight * 3);
			m_transformData.mFrameFmt = CodecImgFormat::FORMAT_BGR;
			m_status = true;
			m_MtxFrame.unlock();
		} else {
			RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"),"Only decoding of JPEG is supported");
      rclcpp::shutdown();
		}
	}
  return 0;
}

int HobotVdec::ReleaseOutput(const std::shared_ptr<OutputFrameDataType>& pFrame)
{
  if (pFrame) {
      RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "[%s] y: 0x%x, uv: 0x%x, w: %d, h: %d",
          __func__,
          pFrame->mPtrY, pFrame->mPtrUV, 
          pFrame->mWidth, pFrame->mHeight);
  }
  return 0;
}

int HobotVdec::GetOutput(std::shared_ptr<OutputFrameDataType> pOutFrm) {
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

int HobotVdec::CheckParams(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para)
{
  if (!sp_hobot_codec_para) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "Invalid input");
    return -1;
  }

  return 0;
}
