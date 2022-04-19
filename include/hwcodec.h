// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/time.h>

#include <string>
#include <queue>
#include <mutex>
#include <list>

#include "vio/hb_comm_venc.h"
#include "vio/hb_venc.h"
#include "vio/hb_vio_interface.h"
#include "vio/hb_sys.h"
#include "vio/hb_vdec.h"
#include "vio/hb_vp_api.h"

#ifndef INCLUDE_HWCODEC_H_
#define INCLUDE_HWCODEC_H_

#define SET_BYTE(_p, _b) \
    *_p++ = (unsigned char)_b;

#define SET_BUFFER(_p, _buf, _len) \
    memcpy(_p, _buf, _len); \
    (_p) += (_len);

typedef enum {
  enCT_STOP = 0,
  enCT_START
};

typedef struct _tag_FrameData {
  uint8_t *mPtrData;
  uint8_t *mPtrY;
  uint8_t *mPtrUV;
  int mDataLen;
  int mFrameFmt;
  int mWidth;
  int mHeight;
  struct timespec time_stamp;
} TFrameData;

#define MMZ_CNT 5
class HWCodec {
 public:
  HWCodec(int channel, const char *type) : m_nCodecSt(enCT_STOP), m_nCodecChn(channel)
  {
    snprintf(m_tsCodecType, sizeof(m_tsCodecType), "%s", type);

    if (strcmp(m_tsCodecType, "h264") == 0)
      m_enPalType = PT_H264;
    else if (strcmp(m_tsCodecType, "h265") == 0)
      m_enPalType = PT_H265;
    else if (strcmp(m_tsCodecType, "jpeg") == 0)
      m_enPalType = PT_JPEG;
  };
  ~HWCodec() {}

  virtual int InitCodec() {}
  virtual int UninitCodec() {}
  int GetFrameNum() {return m_nUseCnt;}
  virtual void SetCodecAttr(const char* tsName, float fVal) {}
  int Start(int nPicWidth, int nPicHeight)
  {
    return child_start(nPicWidth, nPicHeight);
  }
  int Stop()
  {
    return child_stop();
  }
  // 由于硬编解码器只支持nv12 ，所以输入输出都是 nv12 格式
  virtual int PutData(const uint8_t *pDataIn, int nLen, const struct timespec &time_stamp) {
    m_MtxLstFrameStamp.lock();
    m_lstFrameStamp.push_back(time_stamp);
    m_MtxLstFrameStamp.unlock();
    return 0;
  }
  virtual int GetFrame(TFrameData *pOutFrm) {
    m_MtxLstFrameStamp.lock();
    if (m_lstFrameStamp.empty()) {
        m_MtxLstFrameStamp.unlock();
        return -1;
    }
    pOutFrm->time_stamp = m_lstFrameStamp.front();
    m_lstFrameStamp.pop_front();
    m_MtxLstFrameStamp.unlock();
    return 0;
  }
  virtual int ReleaseFrame(TFrameData *pFrame) = 0;

 protected:
  virtual int child_start(int nPicWidth, int nPicHeight) {return 0;}
  virtual int child_stop() {return 0;}

 protected:
  int m_nPicWidth;    /*the attribute of video encoder*/
  int m_nPicHeight;   /*the attribute of rate  ctrl*/
  PAYLOAD_TYPE_E m_enPalType; /*the attribute of gop*/
  pthread_mutex_t m_lckInit;
  pthread_cond_t m_condInit;
  int m_nCodecChn;
  int m_nCodecSt;
  char m_tsCodecType[12] = "";

 protected:
  uint64_t m_arrMMZ_PAddr[MMZ_CNT];
  char* m_arrMMZ_VAddr[MMZ_CNT];
  // memset(mmz_paddr, 0, sizeof(mmz_paddr));
  // int mmz_size;  //= width * height;
  uint32_t m_tmLastPush = 0;  // 上次 push 的时间
  int m_nMMZCnt = MMZ_CNT;
  int m_nUseCnt = 0;  // push 个数
  int m_nGetCnt = 0;  // 读取个数
  int m_nMMZidx = 0;
  // 时间戳队列，不晓得编码、解码出来的时间戳是否对应，先FIFO 处理
  std::mutex          m_MtxLstFrameStamp;
  std::list<struct timespec>  m_lstFrameStamp;
  // 保存编解码后的数据
  std::mutex          m_MtxLstFrames;
  std::list<TFrameData>  m_lstFrames;

 private:
  pthread_t m_pidCodec;
};

#endif  // INCLUDE_HWCODEC_H_
