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

#include<fstream> 
#include "rclcpp/rclcpp.hpp"
#include "hobot_vdec.h"
#include "decoder/hobot_decoder_common.hpp"

int HobotVdec::Init(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para) {
  if (!sp_hobot_codec_para || 0 != CheckParams(sp_hobot_codec_para)) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "Invalid codec para");
    return -1;
  }

  if ("h264" == sp_hobot_codec_para->in_format_)
    m_enPalType = PT_H264;
  else if ("h265" == sp_hobot_codec_para->in_format_)
    m_enPalType = PT_H265;
  else if ("jpeg" == sp_hobot_codec_para->in_format_ ||
  "jpeg-compressed" == sp_hobot_codec_para->in_format_)
    m_enPalType = PT_JPEG;
  else {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "Invalid in_format: %s. %d",
    sp_hobot_codec_para->in_format_.c_str(), __LINE__);
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
  int s32Ret;
  if (CodecStatType::STOP == codec_stat_)
      return 0;
  codec_stat_ = CodecStatType::STOP;
  for (int i = 0; i < m_nMMZCnt; i++) {
      s32Ret = HB_SYS_Free(m_arrMMZ_PAddr[i], m_arrMMZ_VAddr[i]);
      if (s32Ret == 0) {
          RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "mmzFree paddr = 0x%x, vaddr = 0x%x i = %d \n", m_arrMMZ_PAddr[i],
                  m_arrMMZ_VAddr[i], i);
      }
  }
  s32Ret = HB_VP_Exit();
  if (s32Ret == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "vp exit ok!\n");
  }

  s32Ret = HB_VDEC_StopRecvStream(codec_chn_);
  if (s32Ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "HB_VDEC_StopRecvStream failed\n");
      return -1;
  }
  HB_VDEC_ResetChn(codec_chn_);
  s32Ret = HB_VDEC_DestroyChn(codec_chn_);
  if (s32Ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "HB_VDEC_DestroyChn failed\n");
      return -1;
  }

  s32Ret = HB_VDEC_Module_Uninit();
  if (s32Ret) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "HB_VDEC_Module_Uninit: %d\n", s32Ret);
  }
  RCLCPP_INFO(rclcpp::get_logger("HobotVdec"), "Done\n");
  return 0;
}

int HobotVdec::chnAttr_init() {
  // 该步骤必不可少
  memset(&m_oVdecChnAttr, 0, sizeof(VDEC_CHN_ATTR_S));
  // 设置解码模式分别为 PT_H264 PT_H265 PT_MJPEG
  m_oVdecChnAttr.enType = m_enPalType;
  // 设置解码模式为帧模式
  m_oVdecChnAttr.enMode = VIDEO_MODE_FRAME;
  // 设置像素格式 NV12格式
  m_oVdecChnAttr.enPixelFormat = HB_PIXEL_FORMAT_NV12;
  // 输入buffer个数
  m_oVdecChnAttr.u32FrameBufCnt = 3;
  // 输出buffer个数
  m_oVdecChnAttr.u32StreamBufCnt = 3;
  // 输出buffer size，必须1024对齐
  m_oVdecChnAttr.u32StreamBufSize = (init_pic_w_ * init_pic_h_ * 3 / 2 + 1024) & ~0x3ff;
  // 使用外部buffer
  m_oVdecChnAttr.bExternalBitStreamBuf  = HB_TRUE;
  if (m_enPalType == PT_H265) {
      // 使能带宽优化
      m_oVdecChnAttr.stAttrH265.bandwidth_Opt = HB_TRUE;
      // 普通解码模式，IPB帧解码
      m_oVdecChnAttr.stAttrH265.enDecMode = VIDEO_DEC_MODE_NORMAL;
      // 输出顺序按照播放顺序输出
      m_oVdecChnAttr.stAttrH265.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
      // 不启用CLA作为BLA处理
      m_oVdecChnAttr.stAttrH265.cra_as_bla = HB_FALSE;
      // 配置tempral id为绝对模式
      m_oVdecChnAttr.stAttrH265.dec_temporal_id_mode = 0;
      // 保持2
      m_oVdecChnAttr.stAttrH265.target_dec_temporal_id_plus1 = 2;
  } else if (m_enPalType == PT_H264) {
        // 使能带宽优化
      m_oVdecChnAttr.stAttrH264.bandwidth_Opt = HB_TRUE;
      // 普通解码模式，IPB帧解码
      m_oVdecChnAttr.stAttrH264.enDecMode = VIDEO_DEC_MODE_NORMAL;
      // 输出顺序按照播放顺序输出
      m_oVdecChnAttr.stAttrH264.enOutputOrder = VIDEO_OUTPUT_ORDER_DISP;
  } else if (m_enPalType == PT_JPEG) {
      // 使用内部buffer
      m_oVdecChnAttr.bExternalBitStreamBuf  = HB_FALSE;
      // 配置镜像模式，不镜像
      m_oVdecChnAttr.stAttrJpeg.enMirrorFlip = DIRECTION_NONE;
      // 配置旋转模式，不旋转
      m_oVdecChnAttr.stAttrJpeg.enRotation = CODEC_ROTATION_0;
      // 配置crop，不启用
      m_oVdecChnAttr.stAttrJpeg.stCropCfg.bEnable = HB_FALSE;
  }
  RCLCPP_INFO(rclcpp::get_logger("HobotVdec"), "[chnAttr_init] m_enPalType: %d",
  m_enPalType);
  
  return 0;
}

int HobotVdec::init_vdec() {
  int s32Ret;
  pthread_mutex_lock(&m_lckInit);
  // 初始化channel属性
  s32Ret = chnAttr_init();
  if (s32Ret) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "sample_venc_ChnAttr_init failded: %d\n", s32Ret);
  }
  // 创建channel
  s32Ret = HB_VDEC_CreateChn(codec_chn_, &m_oVdecChnAttr);
  if (s32Ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "HB_VDEC_CreateChn %d failed, %x.\n", codec_chn_, s32Ret);
      return -1;
  }
  // 设置channel属性
  s32Ret = HB_VDEC_SetChnAttr(codec_chn_, &m_oVdecChnAttr);  // config
  if (s32Ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "HB_VDEC_SetChnAttr failed\n");
      return -1;
  }

  s32Ret = HB_VDEC_StartRecvStream(codec_chn_);
  if (s32Ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "HB_VDEC_StartRecvStream failed\n");
      return -1;
  }
  pthread_cond_signal(&m_condInit);
  pthread_mutex_unlock(&m_lckInit);

  // 准备buffer
  codec_buf_size_ = init_pic_w_ * init_pic_h_;
  RCLCPP_INFO(rclcpp::get_logger("HobotVdec"), "Alloc buffer count: %d, size: %d, w: %d, h: %d",
  m_nMMZCnt, codec_buf_size_, init_pic_w_, init_pic_h_);
  for (int i = 0; i < m_nMMZCnt; i++) {
      m_arrMMZ_VAddr[i] = NULL;
  }
  memset(m_arrMMZ_PAddr, 0, sizeof(m_arrMMZ_PAddr));
  for (int i = 0; i < m_nMMZCnt; i++) {
      s32Ret = HB_SYS_Alloc(&m_arrMMZ_PAddr[i], reinterpret_cast<void **>(&m_arrMMZ_VAddr[i]), codec_buf_size_);
      if (s32Ret == 0) {
          RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "mmzAlloc paddr = 0x%x, vaddr = 0x%x i = %d",
                  m_arrMMZ_PAddr[i], m_arrMMZ_VAddr[i], i);
      }
  }
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
  // 初始化VP
  VP_CONFIG_S struVpConf;
  memset(&struVpConf, 0x00, sizeof(VP_CONFIG_S));
  struVpConf.u32MaxPoolCnt = 32;
  HB_VP_SetConfig(&struVpConf);
  int s32Ret = HB_VP_Init();
  if (s32Ret != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "vp_init fail s32Ret = %d !\n", s32Ret);
      return s32Ret;
  }
  // codec_chn_ = 0;
  pthread_mutex_init(&m_lckInit, NULL);
  pthread_cond_init(&m_condInit, NULL);
  s32Ret = HB_VDEC_Module_Init();
  if (s32Ret) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "HB_VDEC_Module_Init: %d\n", s32Ret);
      return s32Ret;
  }
  s32Ret = init_vdec();
  if (m_enPalType != PT_JPEG) {
      parse_stream_ = true;
  } else {
      parse_stream_ = false;
  }
  codec_stat_ = CodecStatType::START;
  
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "Start success");
  return s32Ret;
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
  int s32Ret;
  // h26X 前几帧没有 I 帧，解码失败是正常的，只要不崩溃就可以
  if (m_enPalType != PT_JPEG) {
      if (parse_stream_) {
        // 检查是否是I 帧
        int nSPSPos = 0, nPPSPos = 0, nVPSPos = 0;
        int nSPSLen = 0, nPPSLen = 0, nVPSLen = 0;
        RCLCPP_INFO(rclcpp::get_logger("HobotVdec"), "findSPSPPSVPS start");
        int nIFramePos = hobot_decoder_common::findSPSPPSVPS(frame_fmt_, pDataIn,
            nLen, &nSPSPos, &nPPSPos, &nVPSPos, &nSPSLen, &nPPSLen, &nVPSLen);
        if (nSPSLen <= 0 || nIFramePos < 0) {
          RCLCPP_WARN(rclcpp::get_logger("HobotVdec"), "findSPSPPSVPS fail. ret: %d, nSPSLen: %d, nLen: %d",
              nIFramePos, nSPSLen, nLen);
          // rclcpp::shutdown();
          return -1;
        } else {
          RCLCPP_INFO(rclcpp::get_logger("HobotVdec"), "findSPSPPSVPS success");
        }
        // 找到I帧
        parse_stream_ = false;
      }
  }
  VIDEO_FRAME_S pstFrame;
  VIDEO_STREAM_S pstStream;
  memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
  memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));
  m_nMMZidx = m_nUseCnt % m_nMMZCnt;
  VDEC_CHN_STATUS_S pstStatus;
  HB_VDEC_QueryStatus(codec_chn_, &pstStatus);
  if (pstStatus.cur_input_buf_cnt >= (uint32_t)m_nMMZCnt) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "[HB_VDEC_QueryStatus]->dlen=%d, inbufCnt=%d ->%d.\n",
          nLen, pstStatus.cur_input_buf_cnt, m_nMMZCnt);
      usleep(10000);
      return -1;
  }
  memcpy(reinterpret_cast<void*>(m_arrMMZ_VAddr[m_nMMZidx]), pDataIn, nLen);
  pstStream.pstPack.phy_ptr = m_arrMMZ_PAddr[m_nMMZidx];
  pstStream.pstPack.vir_ptr = m_arrMMZ_VAddr[m_nMMZidx];
  pstStream.pstPack.pts = m_nUseCnt++;
  pstStream.pstPack.src_idx = m_nMMZidx;
  pstStream.pstPack.size = nLen;
  pstStream.pstPack.stream_end = HB_FALSE;   // TRUE 就结束
  s32Ret = HB_VDEC_SendStream(codec_chn_, &pstStream, 3000);
  RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "[PutData] pts:%d, vir_ptr:%x, size:%d, ret=%d.\n",
          pstStream.pstPack.pts,
          pstStream.pstPack.vir_ptr,
          pstStream.pstPack.size, s32Ret);
  if (s32Ret == -HB_ERR_VDEC_OPERATION_NOT_ALLOWDED ||
              s32Ret == -HB_ERR_VDEC_UNKNOWN) {
      return -2;
  }
  return 0;
}

int HobotVdec::ReleaseOutput(const std::shared_ptr<OutputFrameDataType>& pFrame)
{
  if (pFrame) {
      // VIDEO_FRAME_S curFrameInfo;
      // curFrameInfo.stVFrame.vir_ptr[0] = (hb_char*)pFrame->mPtrY;
      // curFrameInfo.stVFrame.vir_ptr[1] = (hb_char*)pFrame->mPtrUV;
      RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "[%s] y: 0x%x, uv: 0x%x, w: %d, h: %d",
          __func__,
          pFrame->mPtrY, pFrame->mPtrUV, 
          pFrame->mWidth, pFrame->mHeight);
      return HB_VDEC_ReleaseFrame(codec_chn_, &m_curFrameInfo);
  }
  return 0;
}

int HobotVdec::GetOutput(std::shared_ptr<OutputFrameDataType> pOutFrm) {
  int s32Ret = 0;
  // 解码器是否已经启动，并且已经找到I帧
  if (CodecStatType::START == codec_stat_ && !parse_stream_) {
      s32Ret = HB_VDEC_GetFrame(codec_chn_, &m_curFrameInfo, 1000);
      if (s32Ret == 0) {
          // m_curFrameInfo.stVFrame.height * m_curFrameInfo.stVFrame.width
          pOutFrm->mPtrY = reinterpret_cast<uint8_t*>(m_curFrameInfo.stVFrame.vir_ptr[0]);
          pOutFrm->mPtrUV = reinterpret_cast<uint8_t*>(m_curFrameInfo.stVFrame.vir_ptr[1]);
          // m_curFrameInfo.stVFrame.height * m_curFrameInfo.stVFrame.width / 2
          pOutFrm->mDataLen = m_curFrameInfo.stVFrame.height * m_curFrameInfo.stVFrame.width * 3 / 2;
          pOutFrm->mWidth = m_curFrameInfo.stVFrame.width;
          pOutFrm->mHeight = m_curFrameInfo.stVFrame.height;
          pOutFrm->mFrameFmt = CodecImgFormat::FORMAT_NV12;  // 通通 nv12
          RCLCPP_DEBUG(rclcpp::get_logger("HobotVdec"), "GetFrame w:h=%dx%d, dlen=%d.",
              m_curFrameInfo.stVFrame.width, m_curFrameInfo.stVFrame.height, pOutFrm->mDataLen);
          // FILE *outFile = fopen("decode.nv12", "wb");
          // fwrite(pOutFrm->mPtrY, pOutFrm->mWidth * pOutFrm->mHeight, 1, outFile);
          // fwrite(pOutFrm->mPtrUV, pOutFrm->mWidth * pOutFrm->mHeight / 2, 1, outFile);
          // fclose(outFile);
          return 0;
      } else {
          RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "HB_VDEC_GetFrame failed:%d", s32Ret);
          return -1;
      }
  }

  return s32Ret;
}

int HobotVdec::CheckParams(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para)
{
  if (!sp_hobot_codec_para) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "Invalid input");
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

CodecImgFormat HobotVdec::ConvertPalType(const PAYLOAD_TYPE_E& pal_type) {
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
    RCLCPP_ERROR(rclcpp::get_logger("HobotVdec"), "ConvertPalType fail! Unknown pal_type: %d", pal_type);
  }

  return format;
}
