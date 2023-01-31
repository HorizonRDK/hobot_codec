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

#include "include/hobot_codec_base.h"

#include "vio/hb_comm_venc.h"
#include "vio/hb_venc.h"
#include "vio/hb_vio_interface.h"
#include "vio/hb_sys.h"
#include "vio/hb_vdec.h"
#include "vio/hb_vp_api.h"

#ifndef INCLUDE_HOBOT_VDEC_H_
#define INCLUDE_HOBOT_VDEC_H_

class HobotVdec : public HobotCodecBase {
 public:
  HobotVdec() {}
  ~HobotVdec() {}
  int Init(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para) override;
  int DeInit() override;
  int Start(int nPicWidth, int nPicHeight) override;
  int Stop() override;
  int Input(const uint8_t *pDataIn, int nPicWidth, int nPicHeight, int nLen,
            const struct timespec &time_stamp) override;
  int GetOutput(std::shared_ptr<OutputFrameDataType> pFrame) override;
  int ReleaseOutput(const std::shared_ptr<OutputFrameDataType>& pFrame) override;

 protected:
  int CheckParams(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para) override;

 private:
  int init_vdec();
  int chnAttr_init();
  CodecImgFormat ConvertPalType(const PAYLOAD_TYPE_E& pal_type);

  PAYLOAD_TYPE_E m_enPalType; /*the attribute of gop*/
  CodecImgFormat frame_fmt_ = CodecImgFormat::FORMAT_INVALID;

  VDEC_CHN_ATTR_S m_oVdecChnAttr;
  VIDEO_FRAME_S m_curFrameInfo;
  bool parse_stream_ = true;   // 是否解码第一帧
  
 protected:
  pthread_mutex_t m_lckInit;
  pthread_cond_t m_condInit;

  // 内存管理
  uint64_t m_arrMMZ_PAddr[BUF_CNT];
  char* m_arrMMZ_VAddr[BUF_CNT];
  uint32_t m_tmLastPush = 0;  // 上次 push 的时间
  int m_nMMZCnt = BUF_CNT;
  int m_nUseCnt = 0;  // push 个数
  int m_nGetCnt = 0;  // 读取个数
  int m_nMMZidx = 0;
};

#endif  // INCLUDE_HOBOT_VDEC_H_
