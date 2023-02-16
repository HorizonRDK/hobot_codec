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

int HobotVdec::Init(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para) {
  if (!sp_hobot_codec_para || 0 != CheckParams(sp_hobot_codec_para)) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "Invalid codec para");
    return -1;
  }

  if ("h264" == sp_hobot_codec_para->in_format_)
    m_enPalType = MEDIA_CODEC_ID_H264;
  else if ("h265" == sp_hobot_codec_para->in_format_)
    m_enPalType = MEDIA_CODEC_ID_H265;
  else if ("jpeg" == sp_hobot_codec_para->in_format_ ||
  "jpeg-compressed" == sp_hobot_codec_para->in_format_)
    m_enPalType = MEDIA_CODEC_ID_JPEG;
  else {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "Invalid in_format: %s",
    sp_hobot_codec_para->in_format_.c_str());
    return -1;
  }

  frame_fmt_ = ConvertPalType(m_enPalType);
  codec_chn_ = sp_hobot_codec_para->mChannel_;

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
  if (CodecStatType::STOP == codec_stat_)
    return 0;
  codec_stat_ = CodecStatType::STOP;
  
  if (context_) {
    media_codec_state_t state = MEDIA_CODEC_STATE_NONE;
    hb_mm_mc_get_state(context_, &state);
    if (state != MEDIA_CODEC_STATE_UNINITIALIZED) {
      int ret = hb_mm_mc_stop(context_);
      if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "hb_mm_mc_stop failed, ret: 0x%x", ret);
        return ret;
      }
      ret = hb_mm_mc_release(context_);
      if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "hb_mm_mc_release failed, ret: 0x%x", ret);
        return ret;
      }
    }
    
    free(context_);
    context_ = nullptr;
  }
      
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "Stop success");
  return 0;
}

int HobotVdec::Start(int nPicWidth, int nPicHeight) {
  if (-1 == nPicWidth || -1 == nPicHeight) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "Invalid input w/h");
    return -1;
  }

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
  
  if (MEDIA_CODEC_ID_JPEG == m_enPalType || MEDIA_CODEC_ID_MJPEG == m_enPalType) {
    parse_stream_ = false;
  } else {
    parse_stream_ = true;
  }
  
  if (!context_) {
    context_ = (media_codec_context_t *)malloc(sizeof(media_codec_context_t));
  }
  if (context_ == NULL) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "malloc media_codec_context_t failed");
    return -1;
  }
  memset(context_, 0x00, sizeof(media_codec_context_t));
  context_->codec_id = m_enPalType;
  context_->encoder = false;
  auto params = &context_->video_dec_params;
  params->feed_mode = MC_FEEDING_MODE_FRAME_SIZE;
  params->pix_fmt = MC_PIXEL_FORMAT_NV12;
  params->frame_buf_count = BUF_CNT;
  params->bitstream_buf_count = BUF_CNT;
  if (MEDIA_CODEC_ID_JPEG == m_enPalType ||
  MEDIA_CODEC_ID_MJPEG == m_enPalType) {
    params->jpeg_dec_config.rot_degree = MC_CCW_0;
    params->jpeg_dec_config.mir_direction = MC_DIRECTION_NONE;
    params->jpeg_dec_config.frame_crop_enable = false;
  }

  // codec_buf_size_ = init_pic_w_ * init_pic_h_;
  codec_buf_size_ = (nPicWidth * nPicHeight * 3 / 2 + 0x3ff)&~0x3ff;
  params->bitstream_buf_size = codec_buf_size_;
  RCLCPP_INFO(rclcpp::get_logger("HobotVdec"), "Alloc buffer count: %d, size: %d, w: %d, h: %d",
  params->frame_buf_count, codec_buf_size_, init_pic_w_, init_pic_h_);

  int ret = hb_mm_mc_initialize(context_);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "hb_mm_mc_initialize failed, ret: 0x%x", ret);
    return ret;
  }
  ret = hb_mm_mc_configure(context_);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "hb_mm_mc_configure failed, ret: 0x%x", ret);
    return ret;
  }

  mc_av_codec_startup_params_t startup_params;
  memset(&startup_params, 0x00, sizeof(mc_av_codec_startup_params_t));
  ret = hb_mm_mc_start(context_, &startup_params);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "hb_mm_mc_start failed, ret: 0x%x", ret);
    return ret;
  }

  codec_stat_ = CodecStatType::START;
  
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "Start success");
  return 0;
}

// 如果 h26x 数据，则有 头部处理逻辑
int HobotVdec::Input(const uint8_t *pDataIn, int nPicWidth, int nPicHeight, int nLen, const struct timespec &time_stamp) {
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "Input data w: %d, h: %d, len: %d", nPicWidth, nPicHeight, nLen);

  int ret = Start(nPicWidth, nPicHeight);
  if (ret != 0 ) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"),
      "Start codec failed!");
    return ret;
  }

  // 校验解码器状态
  if (CodecStatType::START != codec_stat_) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"),
    "Codec is not start! codec_stat_: %d", static_cast<int>(codec_stat_));
    return -1;
  }

  // 校验数据大小，收到的数据大小不能超过申请的缓存大小
  if (nLen > codec_buf_size_) {
    std::string fname = std::to_string(time_stamp.tv_sec) + "_" + std::to_string(time_stamp.tv_nsec) + ".jpeg";
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"),
    "input nLen: %d exceeds alloc codec_buf_size_: %d, dump to file: %s",
    nLen, codec_buf_size_, fname.data());
    std::ofstream ofs(fname);
    ofs.write(reinterpret_cast<const char*>(pDataIn), nLen);
    return -1;
  }

  // 解析h26X的I帧
  if (parse_stream_) {
    int nSPSPos = 0, nPPSPos = 0, nVPSPos = 0;
    int nSPSLen = 0, nPPSLen = 0, nVPSLen = 0;
    RCLCPP_INFO(rclcpp::get_logger("HobotVdec"), "findSPSPPSVPS start");
    int nIFramePos = hobot_decoder_common::findSPSPPSVPS(frame_fmt_, pDataIn,
        nLen, &nSPSPos, &nPPSPos, &nVPSPos, &nSPSLen, &nPPSLen, &nVPSLen);
    if (nSPSLen <= 0 || nIFramePos < 0) {
      RCLCPP_WARN(rclcpp::get_logger("HobotVdec"), "findSPSPPSVPS fail. ret: %d, nSPSLen: %d, nLen: %d",
          nIFramePos, nSPSLen, nLen);
      return -1;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("HobotVdec"), "findSPSPPSVPS success");
    }
    // 找到I帧
    parse_stream_ = false;
  }

  /* process input stream */ 
  media_codec_buffer_t inputBuffer;
  memset(&inputBuffer, 0, sizeof(media_codec_buffer_t));
  // 获取输入的buffer
  // 所以使用时直接向vir_ptr地址拷贝图片数据就可以了
  ret = hb_mm_mc_dequeue_input_buffer(context_, &inputBuffer, 3000);
  if (!ret) {
    // 获取输入buffer成功，将图片拷贝到申请的buffer
    memcpy(reinterpret_cast<void*>(inputBuffer.vstream_buf.vir_ptr), pDataIn, nLen);
    inputBuffer.vstream_buf.size = nLen;

    // 填充buffer到codec
    ret = hb_mm_mc_queue_input_buffer(context_, &inputBuffer, 100);
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "hb_mm_mc_queue_input_buffer failed, ret: 0x%x", ret);
      return ret;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "Dequeue input buffer failed, ret = 0x%x", ret);
    return ret;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "Input success");

  return 0;
}

int HobotVdec::ReleaseOutput(const std::shared_ptr<OutputFrameDataType>& pFrame) {
  int ret = hb_mm_mc_queue_output_buffer(context_, &outputBuffer_, 100);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "hb_mm_mc_queue_output_buffer failed, ret: 0x%x", ret);
    return ret;
  }
  if (pFrame && pFrame->sp_frame_info) {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("HobotVdec"),
      "Release output success!"
      << "，" << pFrame->sp_frame_info->img_idx_
      << "，" << pFrame->sp_frame_info->img_ts_.tv_sec
      << "." << pFrame->sp_frame_info->img_ts_.tv_nsec
    );
  }
  return 0;
}

int HobotVdec::GetOutput(std::shared_ptr<OutputFrameDataType> pOutFrm) {
  // 解码器是否已经启动，并且已经找到I帧
  if (CodecStatType::START == codec_stat_ && !parse_stream_) {
    /* process output frame */
    memset(&outputBuffer_, 0, sizeof(media_codec_buffer_t));
    /* dequeue output buffer */ 
    int ret = hb_mm_mc_dequeue_output_buffer(context_, &outputBuffer_, NULL, 3000);
    if (0 == ret) {
      pOutFrm->mPtrY = reinterpret_cast<uint8_t*>(outputBuffer_.vframe_buf.vir_ptr[0]);
      pOutFrm->mPtrUV = reinterpret_cast<uint8_t*>(outputBuffer_.vframe_buf.vir_ptr[1]);
      pOutFrm->mDataLen = outputBuffer_.vframe_buf.size;
      pOutFrm->mWidth = outputBuffer_.vframe_buf.width;
      pOutFrm->mHeight = outputBuffer_.vframe_buf.height;
      pOutFrm->mFrameFmt = CodecImgFormat::FORMAT_NV12;
      RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "GetOutput w:h=%dx%d, dlen=%d.",
          pOutFrm->mWidth, pOutFrm->mHeight, pOutFrm->mDataLen);
      return 0;
    } else {
      return ret;
    }
  }

  return 0;
}

int HobotVdec::CheckParams(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para) {
  if (!sp_hobot_codec_para) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "Invalid input para");
    return -1;
  }

  if ("h264" == sp_hobot_codec_para->in_format_ ||
        "h264" == sp_hobot_codec_para->out_format_) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "H264 is unsupported for J5 platform!");
    return -1;
  }

  if (sp_hobot_codec_para->mChannel_ < 0 || sp_hobot_codec_para->mChannel_ > 3) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"),
        "Invalid channel number: %d! 0~3 are supported, "
        "please check the channel parameter.", sp_hobot_codec_para->mChannel_);
    rclcpp::shutdown();
    return -1;
  }

  return 0;
}

CodecImgFormat HobotVdec::ConvertPalType(const media_codec_id_t& pal_type) {
  CodecImgFormat format = CodecImgFormat::FORMAT_INVALID;
  if (MEDIA_CODEC_ID_H264 == m_enPalType) {
    format = CodecImgFormat::FORMAT_H264;
  } else if (MEDIA_CODEC_ID_H265 == m_enPalType) {
    format = CodecImgFormat::FORMAT_H265;
  } else if (MEDIA_CODEC_ID_JPEG == m_enPalType) {
    format = CodecImgFormat::FORMAT_JPEG;
  } else if (MEDIA_CODEC_ID_MJPEG == m_enPalType) {
    format = CodecImgFormat::FORMAT_MJPEG;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "ConvertPalType fail! Unknown pal_type: %d", pal_type);
  }

  return format;
}
