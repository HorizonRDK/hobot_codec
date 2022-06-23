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

#include "include/hwcodec.h"

#ifndef INCLUDE_HOBOT_VENC_H_
#define INCLUDE_HOBOT_VENC_H_
class HobotVenc : public HWCodec {
 public:
  HobotVenc(int channel, const char *type);
  ~HobotVenc();
  virtual int InitCodec();
  virtual int UninitCodec();
  virtual int PutData(const uint8_t *pDataIn, int nLen,
    const struct timespec &time_stamp);
  virtual int GetFrame(TFrameData *pOutFrm);
  virtual int ReleaseFrame(TFrameData *pFrame);

  int SetFps(int VeChn, int InputFps, int OutputFps);
  virtual void SetCodecAttr(const char* tsName, float fVal);

 protected:
  virtual int child_start(int nPicWidth, int nPicHeight);
  virtual int child_stop();

 private:
  int Create(int nVW, int nVH, int bits);
  int init_venc();
  int chnAttr_init();
  int venc_setRcParam(int bitRate);
  VENC_CHN_ATTR_S m_oVencChnAttr;
  VIDEO_STREAM_S m_curGetStream;

  float m_fJpgQuality;
  float m_fEncQp;

  std::shared_ptr<std::thread> m_spThrdInit;
  // std::atomic<bool> stop_;
  int exec_init();

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
