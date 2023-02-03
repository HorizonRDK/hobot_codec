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

#ifndef INCLUDE_HOBOT_VENC_H_
#define INCLUDE_HOBOT_VENC_H_

class HobotVenc : public HobotCodecBase {
 public:
  HobotVenc() {}
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

 private:

  CodecImgFormat m_enPalType; /*the attribute of gop*/
  CodecImgFormat frame_fmt_ = CodecImgFormat::FORMAT_INVALID;
  OutputFrameDataType m_transformData;
  bool m_status = false;
  uint8_t * m_DataTmp = nullptr;
  std::mutex m_MtxFrame;
	float m_fJpgQuality;
	float m_fEncQp;
  
 protected:

  // 内存管理
  uint32_t m_tmLastPush = 0;  // 上次 push 的时间
  int m_nUseCnt = 0;  // push 个数
  int m_nGetCnt = 0;  // 读取个数
};

#endif  // INCLUDE_HOBOT_VENC_H_
