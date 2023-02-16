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
// H264 H265 MJPEG
#include "include/video_utils.hpp"

HobotVenc::HobotVenc() : m_fJpgQuality(0), m_fEncQp(20.0) {
}

HobotVenc::~HobotVenc() {
}

int HobotVenc::Init(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para) {
  if (!sp_hobot_codec_para || 0 != CheckParams(sp_hobot_codec_para)) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "Invalid codec para");
    return -1;
  }

  if (strcmp(sp_hobot_codec_para->out_format_.data(), "h264") == 0)
    m_enPalType = PT_H264;
  else if (strcmp(sp_hobot_codec_para->out_format_.data(), "h265") == 0)
    m_enPalType = PT_H265;
  else if (strcmp(sp_hobot_codec_para->out_format_.data(), "jpeg") == 0 ||
            strcmp(sp_hobot_codec_para->out_format_.data(), "jpeg-compressed") == 0)
    m_enPalType = PT_JPEG;
  else {
    RCLCPP_INFO(rclcpp::get_logger("HobotVenc"), "Invalid in_format: %s",
    sp_hobot_codec_para->in_format_.c_str());
    return -1;
  }

  frame_fmt_ = ConvertPalType(m_enPalType);

  m_fJpgQuality = sp_hobot_codec_para->jpg_quality_;
  m_fEncQp = sp_hobot_codec_para->enc_qp_;
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
  int s32Ret;
  if (CodecStatType::STOP == codec_stat_)
      return 0;
  codec_stat_ = CodecStatType::STOP;

  for (int i = 0; i < m_nMMZCnt; i++) {
      s32Ret = HB_SYS_Free(m_arrMMZ_PAddr[i], m_arrMMZ_VAddr[i]);
      if (s32Ret == 0) {
          RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "mmzFree paddr = 0x%x, vaddr = 0x%x i = %d ",
            m_arrMMZ_PAddr[i], m_arrMMZ_VAddr[i], i);
      }
  }
  s32Ret = HB_VENC_StopRecvFrame(codec_chn_);
  if (s32Ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "HB_VENC_StopRecvFrame failed");
      return -1;
  }
  s32Ret = HB_VENC_DestroyChn(codec_chn_);
  if (s32Ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "HB_VENC_DestroyChn failed");
      return -1;
  }
  s32Ret = HB_VP_Exit();
  if (s32Ret == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "vp exit ok!");
  }
  s32Ret = HB_VENC_Module_Uninit();
  if (s32Ret) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "HB_VENC_Module_Uninit: %d", s32Ret);
  }

  RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "Stop success");
  return 0;
}

int HobotVenc::Input(const uint8_t *pDataIn, int nPicWidth, int nPicHeight, int nLen, const struct timespec &time_stamp) {
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "Input data w: %d, h: %d, len: %d", nPicWidth, nPicHeight, nLen);

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

  // 先来获取编码
  VIDEO_FRAME_S pstFrame;
  memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
  int offset = init_pic_w_ * init_pic_h_;
  m_nMMZidx = m_nUseCnt % m_nMMZCnt;

  memcpy(reinterpret_cast<void*>(m_arrMMZ_VAddr[m_nMMZidx]), pDataIn, nLen);
  pstFrame.stVFrame.width = init_pic_w_;
  pstFrame.stVFrame.height = init_pic_h_;
  pstFrame.stVFrame.size = nLen;  // init_pic_w_ * init_pic_h_ * 3 / 2;
  pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
  pstFrame.stVFrame.phy_ptr[0] = m_arrMMZ_PAddr[m_nMMZidx];
  pstFrame.stVFrame.phy_ptr[1] = m_arrMMZ_PAddr[m_nMMZidx] + offset;
  // pstFrame.stVFrame.phy_ptr[2] = m_arrMMZ_PAddr[m_nMMZidx] + offset * 5 / 4;
  pstFrame.stVFrame.vir_ptr[0] = m_arrMMZ_VAddr[m_nMMZidx];
  pstFrame.stVFrame.vir_ptr[1] = m_arrMMZ_VAddr[m_nMMZidx] + offset;
  // pstFrame.stVFrame.vir_ptr[2] = m_arrMMZ_VAddr[m_nMMZidx] + offset * 5 / 4;
  pstFrame.stVFrame.pts = time_stamp.tv_sec * 1000 + time_stamp.tv_nsec / 1000000;

  m_nUseCnt++;  // wuwlNG
  if (HB_VENC_SendFrame(codec_chn_, &pstFrame, 3000) != 0) {
      return -1;
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
  int s32Ret;
  if (CodecStatType::START == codec_stat_) {
    s32Ret = HB_VENC_GetStream(codec_chn_, &m_curGetStream, 3000);
    if (s32Ret != 0) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("HobotVenc"),
        "HB_VENC_GetStream fail! s32Ret: " << s32Ret);
      return -1;
    }
    pOutFrm->mPtrData = reinterpret_cast<uint8_t*>(
        m_curGetStream.pstPack.vir_ptr);
    pOutFrm->mDataLen = m_curGetStream.pstPack.size;
    pOutFrm->mWidth = init_pic_w_;
    pOutFrm->mHeight = init_pic_h_;
    pOutFrm->mFrameFmt = frame_fmt_;
    ++m_nGetCnt;
#ifdef TEST_SAVE
    if (NULL == outH264File)
        outH264File = fopen("enc.dat", "wb");
    if (outH264File) {
        fwrite(m_curGetStream.pstPack.vir_ptr,
            m_curGetStream.pstPack.size, 1, outH264File);
    }
#endif
    return 0;
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

  int s32Ret = HB_VENC_ReleaseStream(codec_chn_, &m_curGetStream);
  m_curGetStream.pstPack.vir_ptr = NULL;
  m_curGetStream.pstPack.size = 0;
  if (s32Ret != 0)
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "[%s]0x%x-%dx%d,ret=%d", __func__,
        pFrame->mPtrData, pFrame->mWidth, pFrame->mHeight, s32Ret);
  return s32Ret;
}

int HobotVenc::CheckParams(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para)
{
  if (!sp_hobot_codec_para) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "Invalid input");
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

int HobotVenc::FormalInit()
{
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "FormalInit start");

  int s32Ret;
  pthread_mutex_init(&m_lckInit, NULL);
  pthread_cond_init(&m_condInit, NULL);

  s32Ret = HB_VENC_Module_Init();
  // 初始化VP
  VP_CONFIG_S struVpConf;
  memset(&struVpConf, 0x00, sizeof(VP_CONFIG_S));
  struVpConf.u32MaxPoolCnt = 32;
  HB_VP_SetConfig(&struVpConf);
  s32Ret = HB_VP_Init();

  if (0 != init_venc()) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "init_venc failed");
      HB_VP_Exit();
      HB_VENC_Module_Uninit();
      abort();  // 直接退出
      return -1;
  }

  VENC_RECV_PIC_PARAM_S pstRecvParam;
  pstRecvParam.s32RecvPicNum = 0;  // unchangable

  s32Ret = HB_VENC_StartRecvFrame(codec_chn_, &pstRecvParam);
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "VENC_StartRecvFrame chn=%d,ret=%d.", codec_chn_, s32Ret);
  if (s32Ret != 0) {
      abort();  // 直接退出
      return -2;
  }

  // 准备buffer
  int i = 0;
  for (i = 0; i < m_nMMZCnt; i++) {
      m_arrMMZ_VAddr[i] = NULL;
      m_arrMMZ_PAddr[i] = 0;
  }
  // memset(m_arrMMZ_PAddr, 0, sizeof(m_arrMMZ_PAddr));
  int codec_buf_size_ = init_pic_w_ * init_pic_h_ * 3 / 2;
  for (i = 0; i < m_nMMZCnt; i++) {
      s32Ret = HB_SYS_Alloc(&m_arrMMZ_PAddr[i], reinterpret_cast<void **>(
                  &m_arrMMZ_VAddr[i]), codec_buf_size_);
      if (s32Ret == 0) {
          RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "mmz w:h=%d:%d, paddr=0x%x, vaddr=0x%x i = %d ",
              init_pic_w_, init_pic_h_, m_arrMMZ_PAddr[i],
              m_arrMMZ_VAddr[i], i);
      }
  }

  RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "[%s]: %d end.", __func__, s32Ret);
  
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "FormalInit success");
  return 0;
}

int HobotVenc::init_venc()
{
    int s32Ret;

    pthread_mutex_lock(&m_lckInit);
    // 初始化channel属性
    s32Ret = chnAttr_init();
    RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "sample_venc_ChnAttr_init : %d", s32Ret);
    // 创建channel
    s32Ret = HB_VENC_CreateChn(codec_chn_, &m_oVencChnAttr);
    RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "HB_VENC_CreateChn type=%d, %d , %d.", m_enPalType, codec_chn_, s32Ret);
    if (s32Ret != 0) {
        return -1;
    }
    // 配置Rc参数
    if (m_enPalType == PT_JPEG || m_enPalType == PT_MJPEG)
        s32Ret = venc_setRcParam(8000);
    else
        s32Ret = venc_setRcParam(3000);
    RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "sample_venc_setRcParam ret: %d", s32Ret);

    // 设置channel属性
    s32Ret = HB_VENC_SetChnAttr(codec_chn_, &m_oVencChnAttr);  // config
    RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "HB_VENC_SetChnAttr ret=%d.", s32Ret);
    if (s32Ret != 0) {
        HB_VENC_DestroyChn(codec_chn_);
        pthread_mutex_unlock(&m_lckInit);
        return -1;
    }
    pthread_cond_signal(&m_condInit);
    pthread_mutex_unlock(&m_lckInit);
    return 0;
}

FILE *outH264File = NULL;  // fopen("enc.jpg", "wb");
int HobotVenc::chnAttr_init() {
    int streambufSize = 0;
    // 该步骤必不可少
    memset(&m_oVencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    // 设置编码模型分别为 PT_H264 PT_H265 PT_MJPEG
    m_oVencChnAttr.stVencAttr.enType = m_enPalType;
    // 设置编码分辨率
    m_oVencChnAttr.stVencAttr.u32PicWidth = init_pic_w_;
    m_oVencChnAttr.stVencAttr.u32PicHeight = init_pic_h_;
    // 设置像素格式 NV12格式
    m_oVencChnAttr.stVencAttr.enPixelFormat = HB_PIXEL_FORMAT_NV12;
    // 配置图像镜像属性 无镜像
    m_oVencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    // 配置图像旋转属性 不旋转
    m_oVencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    // 配置图像剪裁属性 不剪裁
    m_oVencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    // 输入图像大小 1024对齐
    streambufSize = (init_pic_w_ * init_pic_h_ * 3 / 2 + 1024) & ~0x3ff;
    // vlc_buf_size为经验值，可以减少RAM使用，如果想使用默认值则保持为0
    if (init_pic_w_ * init_pic_h_ > 2688 * 1522) {
        m_oVencChnAttr.stVencAttr.vlc_buf_size = 7900 * 1024;
    } else if (init_pic_w_ * init_pic_h_ > 1920 * 1080) {
        m_oVencChnAttr.stVencAttr.vlc_buf_size = 4 * 1024 * 1024;
    } else if (init_pic_w_ * init_pic_h_ > 1280 * 720) {
        m_oVencChnAttr.stVencAttr.vlc_buf_size = 2100 * 1024;
    } else if (init_pic_w_ * init_pic_h_ > 704 * 576) {
        m_oVencChnAttr.stVencAttr.vlc_buf_size = 2100 * 1024;
    } else {
        m_oVencChnAttr.stVencAttr.vlc_buf_size = 2048 * 1024;
    }
    if (m_enPalType == PT_JPEG || m_enPalType == PT_MJPEG) {
        // 输出码流个数
        m_oVencChnAttr.stVencAttr.u32BitStreamBufferCount = 1;
        // 输入图像个数
        m_oVencChnAttr.stVencAttr.u32FrameBufferCount = 2;
        m_oVencChnAttr.stVencAttr.bExternalFreamBuffer = HB_TRUE;
        // 关闭DCF
        m_oVencChnAttr.stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
        // 质量系数，越小质量越好
        m_oVencChnAttr.stVencAttr.stAttrJpeg.quality_factor = m_fJpgQuality;  // 0;
        // 配置为0 不建议更改
        m_oVencChnAttr.stVencAttr.stAttrJpeg.restart_interval = 0;
        // 4096对齐
        m_oVencChnAttr.stVencAttr.u32BitStreamBufSize = (streambufSize + 4096) & ~4095;
    } else {
        // 输出码流个数
        m_oVencChnAttr.stVencAttr.u32BitStreamBufferCount = 3;
        // 输入图像个数
        m_oVencChnAttr.stVencAttr.u32FrameBufferCount = 3;
        m_oVencChnAttr.stVencAttr.bExternalFreamBuffer = HB_TRUE;
        m_oVencChnAttr.stVencAttr.u32BitStreamBufSize = streambufSize;
    }

    if (m_enPalType == PT_H265) {
        // 配置编码模式为H265VBR
        m_oVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
        // 不使能QpMap
        m_oVencChnAttr.stRcAttr.stH265Vbr.bQpMapEnable = HB_FALSE;
        // 设置I帧Qp值
        m_oVencChnAttr.stRcAttr.stH265Vbr.u32IntraQp = m_fEncQp;  // 20;
        // 设置I帧间隔
        m_oVencChnAttr.stRcAttr.stH265Vbr.u32IntraPeriod = 60;
        // 设置帧率
        m_oVencChnAttr.stRcAttr.stH265Vbr.u32FrameRate = 30;
    }
    if (m_enPalType == PT_H264) {
        // 配置编码模式为H264VBR
        m_oVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
        // 不使能QpMap
        m_oVencChnAttr.stRcAttr.stH264Vbr.bQpMapEnable = HB_FALSE;
        // 设置I帧Qp值
        m_oVencChnAttr.stRcAttr.stH264Vbr.u32IntraQp = 20;
        // 设置I帧间隔
        m_oVencChnAttr.stRcAttr.stH264Vbr.u32IntraPeriod = 60;
        // 设置帧率
        m_oVencChnAttr.stRcAttr.stH264Vbr.u32FrameRate = 30;
        // h264_profile为0，系统自动配置
        m_oVencChnAttr.stVencAttr.stAttrH264.h264_profile = (VENC_H264_PROFILE_E)0;
        // h264_level为0，系统自动配置
        m_oVencChnAttr.stVencAttr.stAttrH264.h264_level = (HB_H264_LEVEL_E)0;
    }
    // 设置GOP结构
    m_oVencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    // 设置IDR帧类型
    m_oVencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    RCLCPP_DEBUG(rclcpp::get_logger("HobotVenc"), "[%s]->rc=%d, vlcSz=%d, streamSz=%d, cur=%d.",
        __func__, m_oVencChnAttr.stRcAttr.enRcMode,
        m_oVencChnAttr.stVencAttr.vlc_buf_size,
        m_oVencChnAttr.stVencAttr.u32BitStreamBufSize, m_enPalType);
    return 0;
}

int HobotVenc::venc_setRcParam(int bitRate) {
    VENC_RC_ATTR_S *pstRcParam;
    int s32Ret;

    if (m_oVencChnAttr.stVencAttr.enType == PT_H264) {
        pstRcParam = &(m_oVencChnAttr.stRcAttr);
        // 为什么之前是VBR 这里改为CBR 之前的VBR是必须的吗？
        m_oVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        s32Ret = HB_VENC_GetRcParam(codec_chn_, pstRcParam);
        if (s32Ret != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "HB_VENC_GetRcParam failed.");
            return -1;
        }
        // 设置码率
        pstRcParam->stH264Cbr.u32BitRate = bitRate;
        // 设置帧率
        pstRcParam->stH264Cbr.u32FrameRate = 30;
        // 设置I帧间隔
        pstRcParam->stH264Cbr.u32IntraPeriod = 30;
        // 设置VbvBufferSize，与码率、解码器速率有关，一般在1000～5000
        pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
        RCLCPP_INFO(rclcpp::get_logger("HobotVenc"), "[%s]->h264 enRcMode=%d, u32VbvBufSz=%d, bitRate=%d",
            __func__, m_oVencChnAttr.stRcAttr.enRcMode,
            m_oVencChnAttr.stRcAttr.stH264Cbr.u32VbvBufferSize, bitRate);
    } else if (m_oVencChnAttr.stVencAttr.enType == PT_H265) {
        pstRcParam = &(m_oVencChnAttr.stRcAttr);
        m_oVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
        s32Ret = HB_VENC_GetRcParam(codec_chn_, pstRcParam);
        if (s32Ret != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "HB_VENC_GetRcParam failed.");
            return -1;
        }
        // 设置码率
        pstRcParam->stH265Cbr.u32BitRate = bitRate;
        // 设置帧率
        pstRcParam->stH265Cbr.u32FrameRate = 30;
        // 设置I帧间隔
        pstRcParam->stH265Cbr.u32IntraPeriod = 30;
        // 设置VbvBufferSize，与码率、解码器速率有关，一般在1000～5000
        pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
        RCLCPP_INFO(rclcpp::get_logger("HobotVenc"), "[%s]->h265 enRMode=%d, u32VbvBufSz=%d, bitRate=%d ",
            __func__, m_oVencChnAttr.stRcAttr.enRcMode,
            m_oVencChnAttr.stRcAttr.stH265Cbr.u32VbvBufferSize, bitRate);
    }
    return 0;
}

int HobotVenc::venc_h264cbr(VENC_RC_ATTR_S *pstRcParam, int bits,
        int framerate, int intraperiod, int vbvbuf)
{
    pstRcParam->stH264Cbr.u32BitRate = bits;
    pstRcParam->stH264Cbr.u32FrameRate = framerate;
    pstRcParam->stH264Cbr.u32IntraPeriod = intraperiod;
    pstRcParam->stH264Cbr.u32VbvBufferSize = vbvbuf;
    pstRcParam->stH264Cbr.u32IntraQp = 30;
    pstRcParam->stH264Cbr.u32InitialRcQp = 63;
    pstRcParam->stH264Cbr.bMbLevelRcEnable = HB_FALSE;
    pstRcParam->stH264Cbr.u32MaxIQp = 45;
    pstRcParam->stH264Cbr.u32MinIQp = 22;
    pstRcParam->stH264Cbr.u32MaxPQp = 45;
    pstRcParam->stH264Cbr.u32MinPQp = 22;
    pstRcParam->stH264Cbr.u32MaxBQp = 45;
    pstRcParam->stH264Cbr.u32MinBQp = 22;
    pstRcParam->stH264Cbr.bHvsQpEnable = HB_TRUE;
    pstRcParam->stH264Cbr.s32HvsQpScale = 2;
    pstRcParam->stH264Cbr.u32MaxDeltaQp = 10;
    pstRcParam->stH264Cbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int HobotVenc::venc_h264vbr(VENC_RC_ATTR_S *pstRcParam, int intraperiod,
            int intraqp, int framerate)
{
    pstRcParam->stH264Vbr.u32IntraPeriod = intraperiod;
    pstRcParam->stH264Vbr.u32IntraQp = intraqp;
    pstRcParam->stH264Vbr.u32FrameRate = framerate;
    pstRcParam->stH264Vbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int HobotVenc::venc_h264avbr(VENC_RC_ATTR_S *pstRcParam, int bits,
        int framerate, int intraperiod, int vbvbuf)
{
    pstRcParam->stH264AVbr.u32BitRate = bits;
    pstRcParam->stH264AVbr.u32FrameRate = framerate;
    pstRcParam->stH264AVbr.u32IntraPeriod = intraperiod;
    pstRcParam->stH264AVbr.u32VbvBufferSize = vbvbuf;
    pstRcParam->stH264AVbr.u32IntraQp = 30;
    pstRcParam->stH264AVbr.u32InitialRcQp = 63;
    pstRcParam->stH264AVbr.bMbLevelRcEnable = HB_FALSE;
    pstRcParam->stH264AVbr.u32MaxIQp = 45;
    pstRcParam->stH264AVbr.u32MinIQp = 22;
    pstRcParam->stH264AVbr.u32MaxPQp = 45;
    pstRcParam->stH264AVbr.u32MinPQp = 22;
    pstRcParam->stH264AVbr.u32MaxBQp = 45;
    pstRcParam->stH264AVbr.u32MinBQp = 22;
    pstRcParam->stH264AVbr.bHvsQpEnable = HB_TRUE;
    pstRcParam->stH264AVbr.s32HvsQpScale = 2;
    pstRcParam->stH264AVbr.u32MaxDeltaQp = 10;
    pstRcParam->stH264AVbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int HobotVenc::venc_h264fixqp(VENC_RC_ATTR_S *pstRcParam, int framerate,
            int intraperiod, int iqp, int pqp, int bqp)
{
    pstRcParam->stH264FixQp.u32FrameRate = framerate;
    pstRcParam->stH264FixQp.u32IntraPeriod = intraperiod;
    pstRcParam->stH264FixQp.u32IQp = iqp;
    pstRcParam->stH264FixQp.u32PQp = pqp;
    pstRcParam->stH264FixQp.u32BQp = bqp;

    return 0;
}

int HobotVenc::venc_h264qpmap(VENC_RC_ATTR_S *pstRcParam, int framerate,
                int intraperiod)
{
    pstRcParam->stH264QpMap.u32IntraPeriod = intraperiod;
    pstRcParam->stH264QpMap.u32FrameRate = framerate;
    pstRcParam->stH264QpMap.u32QpMapArrayCount =
        ((640+0x0f)&~0x0f)/16 * ((480+0x0f)&~0x0f)/16;
    pstRcParam->stH264QpMap.u32QpMapArray =
        (unsigned char*)malloc((pstRcParam->stH264QpMap. \
        u32QpMapArrayCount)*sizeof(unsigned char));
    memset(pstRcParam->stH264QpMap.u32QpMapArray, 30,
            pstRcParam->stH264QpMap.u32QpMapArrayCount);

    return 0;
}

int HobotVenc::venc_h265cbr(VENC_RC_ATTR_S *pstRcParam, int bits,
        int framerate, int intraperiod, int vbvbuf)
{
    pstRcParam->stH265Cbr.u32IntraPeriod = intraperiod;
    pstRcParam->stH265Cbr.u32IntraQp = 30;
    pstRcParam->stH265Cbr.u32BitRate = bits;
    pstRcParam->stH265Cbr.u32FrameRate = framerate;
    pstRcParam->stH265Cbr.u32InitialRcQp = 63;
    pstRcParam->stH265Cbr.u32VbvBufferSize = vbvbuf;
    pstRcParam->stH265Cbr.bCtuLevelRcEnable = HB_FALSE;
    pstRcParam->stH265Cbr.u32MaxIQp = 45;
    pstRcParam->stH265Cbr.u32MinIQp = 22;
    pstRcParam->stH265Cbr.u32MaxPQp = 45;
    pstRcParam->stH265Cbr.u32MinPQp = 22;
    pstRcParam->stH265Cbr.u32MaxBQp = 45;
    pstRcParam->stH265Cbr.u32MinBQp = 22;
    pstRcParam->stH265Cbr.bHvsQpEnable = HB_TRUE;
    pstRcParam->stH265Cbr.s32HvsQpScale = 2;
    pstRcParam->stH265Cbr.u32MaxDeltaQp = 10;
    pstRcParam->stH265Cbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int HobotVenc::venc_h265vbr(VENC_RC_ATTR_S *pstRcParam, int intraperiod,
            int intraqp, int framerate)
{
    pstRcParam->stH265Vbr.u32IntraPeriod = intraperiod;
    pstRcParam->stH265Vbr.u32IntraQp = intraqp;
    pstRcParam->stH265Vbr.u32FrameRate = framerate;
    pstRcParam->stH265Vbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int HobotVenc::venc_h265avbr(VENC_RC_ATTR_S *pstRcParam, int bits,
        int framerate, int intraperiod, int vbvbuf)
{
    pstRcParam->stH265Cbr.u32IntraPeriod = intraperiod;
    pstRcParam->stH265Cbr.u32IntraQp = 30;
    pstRcParam->stH265Cbr.u32BitRate = bits;
    pstRcParam->stH265Cbr.u32FrameRate = framerate;
    pstRcParam->stH265Cbr.u32InitialRcQp = 63;
    pstRcParam->stH265Cbr.u32VbvBufferSize = vbvbuf;
    pstRcParam->stH265Cbr.bCtuLevelRcEnable = HB_FALSE;
    pstRcParam->stH265Cbr.u32MaxIQp = 45;
    pstRcParam->stH265Cbr.u32MinIQp = 22;
    pstRcParam->stH265Cbr.u32MaxPQp = 45;
    pstRcParam->stH265Cbr.u32MinPQp = 22;
    pstRcParam->stH265Cbr.u32MaxBQp = 45;
    pstRcParam->stH265Cbr.u32MinBQp = 22;
    pstRcParam->stH265Cbr.bHvsQpEnable = HB_TRUE;
    pstRcParam->stH265Cbr.s32HvsQpScale = 2;
    pstRcParam->stH265Cbr.u32MaxDeltaQp = 10;
    pstRcParam->stH265Cbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int HobotVenc::venc_h265fixqp(VENC_RC_ATTR_S *pstRcParam, int framerate,
            int intraperiod, int iqp, int pqp, int bqp)
{
    pstRcParam->stH265FixQp.u32FrameRate = framerate;
    pstRcParam->stH265FixQp.u32IntraPeriod = intraperiod;
    pstRcParam->stH265FixQp.u32IQp = iqp;
    pstRcParam->stH265FixQp.u32PQp = pqp;
    pstRcParam->stH265FixQp.u32BQp = bqp;

    return 0;
}

int HobotVenc::venc_h265qpmap(VENC_RC_ATTR_S *pstRcParam, int framerate,
                int intraperiod)
{
    pstRcParam->stH264QpMap.u32IntraPeriod = intraperiod;
    pstRcParam->stH264QpMap.u32FrameRate = framerate;
    pstRcParam->stH264QpMap.u32QpMapArrayCount =
        ((640+0x0f)&~0x0f)/16 * ((480+0x0f)&~0x0f)/16;
    pstRcParam->stH264QpMap.u32QpMapArray =
        (unsigned char*)malloc((pstRcParam->stH264QpMap. \
        u32QpMapArrayCount)*sizeof(unsigned char));
    memset(pstRcParam->stH264QpMap.u32QpMapArray, 30,
            pstRcParam->stH264QpMap.u32QpMapArrayCount);

    return 0;
}

int HobotVenc::venc_mjpgfixqp(VENC_RC_ATTR_S *pstRcParam, int framerate,
                int quality)
{
    pstRcParam->stMjpegFixQp.u32FrameRate = framerate;
    pstRcParam->stMjpegFixQp.u32QualityFactort = quality;
    RCLCPP_INFO(rclcpp::get_logger("HobotVenc"), "[%s]->fps=%d, quality=%d.", __func__, framerate, quality);
    return 0;
}

CodecImgFormat HobotVenc::ConvertPalType(const PAYLOAD_TYPE_E& pal_type) {
  CodecImgFormat format = CodecImgFormat::FORMAT_INVALID;
  if (PT_H264 == m_enPalType) {
    format = CodecImgFormat::FORMAT_H264;
  } else if (PT_H265 == m_enPalType) {
    format = CodecImgFormat::FORMAT_H265;
  } else if (PT_JPEG == m_enPalType) {
    format = CodecImgFormat::FORMAT_JPEG;
  } else if (PT_MJPEG == m_enPalType) {
    format = CodecImgFormat::FORMAT_MJPEG;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVenc"), "ConvertPalType fail! Unknown pal_type: %d", pal_type);
  }

  return format;
}