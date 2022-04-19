// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

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
