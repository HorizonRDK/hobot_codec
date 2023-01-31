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

#include <thread>
#include <functional>
#include <memory>

#include "include/hobot_codec_base.h"

#include "vio/hb_comm_venc.h"
#include "vio/hb_venc.h"
#include "vio/hb_vio_interface.h"
#include "vio/hb_sys.h"
#include "vio/hb_vdec.h"
#include "vio/hb_vp_api.h"

#ifndef INCLUDE_HOBOT_VENC_H_
#define INCLUDE_HOBOT_VENC_H_

// 缓存数量
#define BUF_CNT 5

class HobotVenc : public HobotCodecBase {
 public:
  HobotVenc();
  ~HobotVenc();
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


 private:
  int init_venc();
  int chnAttr_init();
  int venc_setRcParam(int bitRate);
  CodecImgFormat ConvertPalType(const PAYLOAD_TYPE_E& pal_type);

  VENC_CHN_ATTR_S m_oVencChnAttr;
  VIDEO_STREAM_S m_curGetStream;

  PAYLOAD_TYPE_E m_enPalType; /*the attribute of gop*/
  CodecImgFormat frame_fmt_ = CodecImgFormat::FORMAT_INVALID;

  float m_fJpgQuality;
  float m_fEncQp;

  std::shared_ptr<std::thread> m_spThrdInit;
  
  // 真正的初始化，Start中获取到输入分辨率后进行
  int FormalInit();

  // VENC_CHN m_oVeChn;
  int VencChnAttrInit(VENC_CHN_ATTR_S *pVencChnAttr, PAYLOAD_TYPE_E p_enType,
            int p_Width, int p_Height, PIXEL_FORMAT_E pixFmt);
  int venc_h264cbr(VENC_RC_ATTR_S *pstRcParam, int bits,
        int framerate, int intraperiod, int vbvbuf);
  int venc_h264vbr(VENC_RC_ATTR_S *pstRcParam, int intraperiod,
            int intraqp, int framerate);
  int venc_h264avbr(VENC_RC_ATTR_S *pstRcParam, int bits,
        int framerate, int intraperiod, int vbvbuf);
  int venc_h264fixqp(VENC_RC_ATTR_S *pstRcParam, int framerate,
            int intraperiod, int iqp, int pqp, int bqp);
  int venc_h264qpmap(VENC_RC_ATTR_S *pstRcParam, int framerate,
                int intraperiod);
  int venc_h265cbr(VENC_RC_ATTR_S *pstRcParam, int bits,
        int framerate, int intraperiod, int vbvbuf);
  int venc_h265vbr(VENC_RC_ATTR_S *pstRcParam, int intraperiod,
            int intraqp, int framerate);
  int venc_h265avbr(VENC_RC_ATTR_S *pstRcParam, int bits,
        int framerate, int intraperiod, int vbvbuf);
  int venc_h265fixqp(VENC_RC_ATTR_S *pstRcParam, int framerate,
            int intraperiod, int iqp, int pqp, int bqp);
  int venc_h265qpmap(VENC_RC_ATTR_S *pstRcParam, int framerate,
                int intraperiod);
  int venc_mjpgfixqp(VENC_RC_ATTR_S *pstRcParam, int framerate,
                int quality);
};

#endif  // INCLUDE_HOBOT_VENC_H_
