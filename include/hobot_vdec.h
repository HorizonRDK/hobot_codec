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

#include "include/hwcodec.h"

#ifndef INCLUDE_HOBOT_VDEC_H_
#define INCLUDE_HOBOT_VDEC_H_
class HobotVdec : public HWCodec {
 public:
  HobotVdec(int channel, const char *type);
  ~HobotVdec();
  virtual int InitCodec();
  virtual int UninitCodec();
  virtual int PutData(const uint8_t *pDataIn, int nLen, const struct timespec &time_stamp);
  virtual int GetFrame(TFrameData *pOutFrm);
  virtual int ReleaseFrame(TFrameData *pFrame);

 protected:
  virtual int child_start(int nPicWidth, int nPicHeight);
  virtual int child_stop();
 private:
  int init_vdec();
  int chnAttr_init();
  int build_dec_seq_header(uint8_t * pbHeader, int* sizelength, uint8_t* pbMetaData, int nMetaDLen);
  VDEC_CHN_ATTR_S m_oVdecChnAttr;
  VIDEO_FRAME_S m_curFrameInfo;
  int m_bFirstDec;   // 是否解码第一帧
};

#endif  // INCLUDE_HOBOT_VDEC_H_
