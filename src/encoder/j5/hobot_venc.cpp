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
#include "rclcpp/logging.hpp"

#include "hobot_venc.h"
#include "include/video_utils.hpp"

HobotVenc::HobotVenc() {
}

HobotVenc::~HobotVenc() {
}

int HobotVenc::Init(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para) {
  if (!sp_hobot_codec_para || 0 != CheckParams(sp_hobot_codec_para)) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "Invalid codec para");
    return -1;
  }

  if (strcmp(sp_hobot_codec_para->out_format_.data(), "h264") == 0)
    m_enPalType = MEDIA_CODEC_ID_H264;
  else if (strcmp(sp_hobot_codec_para->out_format_.data(), "h265") == 0)
    m_enPalType = MEDIA_CODEC_ID_H265;
  else if (strcmp(sp_hobot_codec_para->out_format_.data(), "jpeg") == 0 ||
            strcmp(sp_hobot_codec_para->out_format_.data(), "jpeg-compressed") == 0)
    m_enPalType = MEDIA_CODEC_ID_JPEG;
  else {
    RCLCPP_INFO(rclcpp::get_logger("HobotVenc"), "Invalid in_format: %s",
    sp_hobot_codec_para->in_format_.c_str());
    return -1;
  }

  frame_fmt_ = ConvertPalType(m_enPalType);
  codec_chn_ = sp_hobot_codec_para->mChannel_;

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

int HobotVenc::Start(int nPicWidth, int nPicHeight) {
  if (-1 == nPicWidth || -1 == nPicHeight) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "Invalid input w/h");
    return -1;
  }

  if (CodecStatType::START == codec_stat_) {
    // 已经初始化过
    // 不同分辨率直接失败不处理
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
  } else {
    init_pic_w_ = nPicWidth;
    init_pic_h_ = nPicHeight;
    if (0 != FormalInit()) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "FormalInit fail!");
      return -1;
    }
  }

  codec_stat_ = CodecStatType::START;

  RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "Start success");
  return 0;
}

int HobotVenc::Stop() {
  if (CodecStatType::STOP == codec_stat_)
    return 0;
    
  if (context_) {
    media_codec_state_t state = MEDIA_CODEC_STATE_NONE; 
    hb_mm_mc_get_state(context_, &state);
    if (state != MEDIA_CODEC_STATE_UNINITIALIZED) {
      int ret = hb_mm_mc_stop(context_); 
      if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "hb_mm_mc_stop failed, ret: 0x%x", ret);
        return ret; 
      }
      ret = hb_mm_mc_release(context_); 
      if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "hb_mm_mc_release failed, ret: 0x%x", ret); 
        return ret;
      }
    }

    free(context_);
    context_ = nullptr;
  }

  codec_stat_ = CodecStatType::STOP;
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "Stop success");
  return 0;
}

int HobotVenc::Input(const uint8_t *pDataIn, int nPicWidth, int nPicHeight, int nLen, const struct timespec &time_stamp) {
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"),
  "Input data ts: %s, w: %d, h: %d, len: %d",
  (std::to_string(time_stamp.tv_sec) + "." + std::to_string(time_stamp.tv_nsec)).data(),
  nPicWidth, nPicHeight, nLen);

  int ret = Start(nPicWidth, nPicHeight);
  if (ret != 0 ) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"),
      "Start codec failed!");
    return ret;
  }

  if (CodecStatType::START != codec_stat_) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("HobotVenc"),
      "Input fail! codec is not ready! codec_stat_: " << static_cast<int>(codec_stat_));
    return -1;
  }

  /* process input stream */ 
  media_codec_buffer_t inputBuffer;
  memset(&inputBuffer, 0, sizeof(media_codec_buffer_t));
  // 获取输入的buffer
  ret = hb_mm_mc_dequeue_input_buffer(context_, &inputBuffer, 3000);
  if (!ret) {
    // 获取输入buffer成功，将图片拷贝到申请的buffer
    int lumaSize = 0, chromaSize = 0, chromaCbSize = 0, chromaCrSize = 0;
    mc_pixel_format_t pix_fmt = context_->video_enc_params.pix_fmt;
    hb_s32 width = context_->video_enc_params.width;
    hb_s32 height = context_->video_enc_params.height;
    hb_u8 *input_virY = inputBuffer.vframe_buf.vir_ptr[0];
    hb_u8 *input_virCb = inputBuffer.vframe_buf.vir_ptr[1];
    hb_u8 *input_virCr = inputBuffer.vframe_buf.vir_ptr[2];

    lumaSize = width * height * byte_per_pixel_;
    chromaSize = lumaSize/2;
    const uint8_t *data_buf = pDataIn;
    memcpy(reinterpret_cast<void*>(input_virY), data_buf, lumaSize);
    data_buf = data_buf + lumaSize;

    // 送的图片是MC_PIXEL_FORMAT_NV12格式
    if (pix_fmt == MC_PIXEL_FORMAT_YUV420P) {
      chromaCbSize = chromaCrSize = chromaSize/2;
      memcpy(reinterpret_cast<void*>(input_virCb), data_buf, chromaCbSize);
      data_buf = data_buf + chromaCbSize;
      memcpy(reinterpret_cast<void*>(input_virCr), data_buf, chromaCrSize);
      data_buf = data_buf + chromaCrSize;
    } else {
      memcpy(reinterpret_cast<void*>(input_virCb), data_buf, chromaSize);
      data_buf = data_buf + chromaSize;
    }

    // 填充buffer到codec
    ret = hb_mm_mc_queue_input_buffer(context_, &inputBuffer, 100);
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "hb_mm_mc_queue_input_buffer failed, ret: 0x%x", ret);
      return ret;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "Dequeue input buffer failed, ret = 0x%x.\n", ret);
    if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
      return ret;
    }
  }
  
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "Input success");
  return 0;
}

int HobotVenc::GetOutput(std::shared_ptr<OutputFrameDataType> pOutFrm) {
  if (!pOutFrm) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("HobotVenc"),
      "Invalid input data!");
    return -1;
  }
  
  if (CodecStatType::START == codec_stat_) {
    /* process output frame */
    memset(&outputBuffer_, 0, sizeof(media_codec_buffer_t));
    /* dequeue output buffer */ 
    int ret = hb_mm_mc_dequeue_output_buffer(context_, &outputBuffer_, NULL, 3000);
    if (0 == ret) {
      pOutFrm->mPtrData = reinterpret_cast<uint8_t*>(outputBuffer_.vstream_buf.vir_ptr);
      pOutFrm->mDataLen = outputBuffer_.vstream_buf.size;
      pOutFrm->mWidth = init_pic_w_;
      pOutFrm->mHeight = init_pic_h_;
      pOutFrm->mFrameFmt = frame_fmt_;
      return 0;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "dequeue output buffer failed(0x%x).\n", ret);
      if (ret != (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
        return ret;
      }
    }
  } else {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("HobotVenc"),
      "GetOutput fail! codec is not ready! codec_stat_: " << static_cast<int>(codec_stat_));
    return -1;
  }

  return -1;
}

int HobotVenc::ReleaseOutput(const std::shared_ptr<OutputFrameDataType>& pFrame) {
  if (!pFrame) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("HobotVenc"),
      "Invalid input data!");
    return -1;
  }

  int ret = hb_mm_mc_queue_output_buffer(context_, &outputBuffer_, 100);
  if (ret) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "hb_mm_mc_queue_output_buffer failed, ret: 0x%x", ret);
    return ret;
  }
  if (outputBuffer_.vstream_buf.stream_end) {
    return ret;
  }

  return 0;
}

int HobotVenc::CheckParams(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para) {
  if (!sp_hobot_codec_para) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "Invalid input");
    return -1;
  }

  if ("h264" == sp_hobot_codec_para->in_format_ ||
        "h264" == sp_hobot_codec_para->out_format_) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "H264 is unsupported for J5 platform!");
    return -1;
  }
  
  if (sp_hobot_codec_para->mChannel_ < 0 || sp_hobot_codec_para->mChannel_ > 3) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"),
        "Invalid channel number: %d! 0~3 are supported, "
        "please check the channel parameter.", sp_hobot_codec_para->mChannel_);
    rclcpp::shutdown();
    return -1;
  }

  if (sp_hobot_codec_para->enc_qp_ < 0 || sp_hobot_codec_para->enc_qp_ > 100) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"),
    "Invalid enc_qp: %f! The value range is floating point number from 0 to 100."
    " Please check the enc_qp parameter.", sp_hobot_codec_para->enc_qp_);
    rclcpp::shutdown();
    return -1;
  }
  if (sp_hobot_codec_para->jpg_quality_ < 0 || sp_hobot_codec_para->jpg_quality_ > 100) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"),
    "Invalid jpg_quality: %f! The value range is floating point number from 0 to 100."
    " Please check the jpg_quality parameter.", sp_hobot_codec_para->jpg_quality_);
    rclcpp::shutdown();
    return -1;
  }

  return 0;
}

int HobotVenc::FormalInit() {
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "FormalInit start");

  if (!context_) {
    context_ = (media_codec_context_t *)malloc(sizeof(media_codec_context_t));
  }
  if (!context_) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "malloc media_codec_context_t failed");
    return -1;
  }
  memset(context_, 0x00, sizeof(media_codec_context_t));
  context_->codec_id = m_enPalType;
  context_->encoder = true;
  auto params = &context_->video_enc_params;
  params->pix_fmt = MC_PIXEL_FORMAT_NV12;
  params->frame_buf_count = BUF_CNT;
  params->bitstream_buf_count = BUF_CNT;
  params->rot_degree = MC_CCW_0;
  params->mir_direction = MC_DIRECTION_NONE;
  params->frame_cropping_flag = false;
  params->width = init_pic_w_;
  params->height = init_pic_h_;
  params->external_frame_buf = false;
  int ret = 0;
  if (CodecImgFormat::FORMAT_H265 == frame_fmt_) {
    params->rc_params.mode = MC_AV_RC_MODE_H265CBR;
    ret = hb_mm_mc_get_rate_control_config(context_, &params->rc_params);
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "hb_mm_mc_get_rate_control_config failed, ret: 0x%x", ret);
      return ret;
    }
    params->rc_params.h265_cbr_params.intra_period = 20;
    params->rc_params.h265_cbr_params.intra_qp = 8;
    params->rc_params.h265_cbr_params.bit_rate = 200;
    params->rc_params.h265_cbr_params.frame_rate = 15;
    params->rc_params.h265_cbr_params.initial_rc_qp = 20;
    params->rc_params.h265_cbr_params.vbv_buffer_size = 20;
    params->rc_params.h265_cbr_params.ctu_level_rc_enalbe = 1;
    params->rc_params.h265_cbr_params.min_qp_I = 8;
    params->rc_params.h265_cbr_params.max_qp_I = 30;
    params->rc_params.h265_cbr_params.min_qp_P = 8;
    params->rc_params.h265_cbr_params.max_qp_P = 30;
    params->rc_params.h265_cbr_params.min_qp_B = 8;
    params->rc_params.h265_cbr_params.max_qp_B = 30;
    params->rc_params.h265_cbr_params.hvs_qp_enable = 1;
    params->rc_params.h265_cbr_params.hvs_qp_scale = 2;
    params->rc_params.h265_cbr_params.max_delta_qp = 10;
    params->rc_params.h265_cbr_params.qp_map_enable = 0;
    params->gop_params.decoding_refresh_type = 2;
    params->gop_params.gop_preset_idx = 2;
  } else if (CodecImgFormat::FORMAT_H264 == frame_fmt_) {
    params->rc_params.mode = MC_AV_RC_MODE_H264CBR;
    ret = hb_mm_mc_get_rate_control_config(context_, &params->rc_params);
    if (ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "hb_mm_mc_get_rate_control_config failed, ret: 0x%x", ret);
      return ret;
    }
    params->rc_params.h264_cbr_params.intra_period = 30;
    params->rc_params.h264_cbr_params.intra_qp = 30;
    params->rc_params.h264_cbr_params.bit_rate = 5000;
    params->rc_params.h264_cbr_params.frame_rate = 30;
    params->rc_params.h264_cbr_params.initial_rc_qp = 20;
    params->rc_params.h264_cbr_params.vbv_buffer_size = 20;
    params->rc_params.h264_cbr_params.mb_level_rc_enalbe = 1;
    params->rc_params.h264_cbr_params.min_qp_I = 8;
    params->rc_params.h264_cbr_params.max_qp_I = 50;
    params->rc_params.h264_cbr_params.min_qp_P = 8;
    params->rc_params.h264_cbr_params.max_qp_P = 50;
    params->rc_params.h264_cbr_params.min_qp_B = 8;
    params->rc_params.h264_cbr_params.max_qp_B = 50;
    params->rc_params.h264_cbr_params.hvs_qp_enable = 1;
    params->rc_params.h264_cbr_params.hvs_qp_scale = 2;
    params->rc_params.h264_cbr_params.max_delta_qp = 10;
    params->rc_params.h264_cbr_params.qp_map_enable = 0;
  }

  ret = hb_mm_mc_initialize(context_);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "hb_mm_mc_initialize failed, ret: 0x%x", ret);
    return ret;
  }

  ret = hb_mm_mc_configure(context_);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "hb_mm_mc_configure failed, ret: 0x%x", ret);
    return ret;
  }

  mc_av_codec_startup_params_t startup_params;
  memset(&startup_params, 0x00, sizeof(mc_av_codec_startup_params_t));
  ret = hb_mm_mc_start(context_, &startup_params);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "hb_mm_mc_start failed, ret: 0x%x", ret);
    return ret;
  }

  return 0;
}

CodecImgFormat HobotVenc::ConvertPalType(const media_codec_id_t& pal_type) {
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
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "ConvertPalType fail! Unknown pal_type: %d", pal_type);
  }

  return format;
}