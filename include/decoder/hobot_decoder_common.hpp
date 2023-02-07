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

#include <string>
#include <queue>
#include <memory>
#include <include/hobot_codec_data.h>

#ifndef INCLUDE_HOBOT_DECODEC_COMMON_H_
#define INCLUDE_HOBOT_DECODEC_COMMON_H_

namespace hobot_decoder_common
{
// 按位与提取NALU类型的掩码位
#define H264_GET_NALU_TYPE 0x1F
// 原始的Nal数据
// H264_I_FRAME        = 0x05,
// H264_P_FRAME        = 0x01,
// H264_SPS_FRAME      = 0x07,
// H264_PPS_FRAME      = 0x08,
// H264_SEI_FRAME      = 0x06,
  
// 按位与提取NALU类型的掩码位
#define H265_GET_NALU_TYPE 0x7E
// 原始的Nal数据
// H265_I_FRAME        = 0x26,
// H265_I_N_LP_FRAME   = 0x28,
// H265_SS_FRAME       = 0x02,
// H265_VPS_FRAME      = 0x40,
// H265_SPS_FRAME      = 0x42,
// H265_PPS_FRAME      = 0x44,
// H265_SEI_FRAME      = 0x4E,

// H264定义的nal数据标志：https://github.com/cisco/openh264/blob/master/codec/api/wels/codec_def.h
// 转换方法：NalUnitType = (NalFrameval & H264_GET_NALU_TYPE);
enum class NalUnitTypeH264 {
  NAL_UNKNOWN     = 0,
  NAL_SLICE       = 1,
  NAL_SLICE_DPA   = 2,
  NAL_SLICE_DPB   = 3,
  NAL_SLICE_DPC   = 4,
  NAL_SLICE_IDR   = 5,      ///< ref_idc != 0
  NAL_SEI         = 6,      ///< ref_idc == 0
  NAL_SPS         = 7,
  NAL_PPS         = 8
                    ///< ref_idc == 0 for 6,9,10,11,12
};

// H265定义的nal数据标志：https://github.com/strukturag/libde265/blob/master/libde265/nal.h
// 转换方法：NalUnitType = (NalFrameval & H265_GET_NALU_TYPE) >> 1;
enum class NalUnitTypeH265 {
  NAL_UNIT_TRAIL_N = 0,
  NAL_UNIT_TRAIL_R = 1,
  NAL_UNIT_TSA_N = 2,
  NAL_UNIT_TSA_R = 3,
  NAL_UNIT_STSA_N = 4,
  NAL_UNIT_STSA_R = 5,
  NAL_UNIT_RADL_N = 6,
  NAL_UNIT_RADL_R = 7,
  NAL_UNIT_RASL_N = 8,
  NAL_UNIT_RASL_R = 9,
  NAL_UNIT_RESERVED_VCL_N10 = 10,
  NAL_UNIT_RESERVED_VCL_N12 = 12,
  NAL_UNIT_RESERVED_VCL_N14 = 14,
  NAL_UNIT_RESERVED_VCL_R11 = 11,
  NAL_UNIT_RESERVED_VCL_R13 = 13,
  NAL_UNIT_RESERVED_VCL_R15 = 15,
  NAL_UNIT_BLA_W_LP = 16,     // BLA = broken link access
  NAL_UNIT_BLA_W_RADL = 17,
  NAL_UNIT_BLA_N_LP = 18,
  NAL_UNIT_IDR_W_RADL = 19,
  NAL_UNIT_IDR_N_LP = 20,
  NAL_UNIT_CRA_NUT = 21,     // CRA = clean random access
  NAL_UNIT_RESERVED_IRAP_VCL22 = 22,
  NAL_UNIT_RESERVED_IRAP_VCL23 = 23,
  NAL_UNIT_RESERVED_VCL24 = 24,
  NAL_UNIT_RESERVED_VCL25 = 25,
  NAL_UNIT_RESERVED_VCL26 = 26,
  NAL_UNIT_RESERVED_VCL27 = 27,
  NAL_UNIT_RESERVED_VCL28 = 28,
  NAL_UNIT_RESERVED_VCL29 = 29,
  NAL_UNIT_RESERVED_VCL30 = 30,
  NAL_UNIT_RESERVED_VCL31 = 31,
  NAL_UNIT_VPS_NUT = 32,
  NAL_UNIT_SPS_NUT = 33,
  NAL_UNIT_PPS_NUT = 34,
  NAL_UNIT_AUD_NUT = 35,
  NAL_UNIT_EOS_NUT = 36,
  NAL_UNIT_EOB_NUT = 37,
  NAL_UNIT_FD_NUT = 38,
  NAL_UNIT_PREFIX_SEI_NUT = 39,
  NAL_UNIT_SUFFIX_SEI_NUT = 40,
  NAL_UNIT_RESERVED_NVCL41 = 41,
  NAL_UNIT_RESERVED_NVCL42 = 42,
  NAL_UNIT_RESERVED_NVCL43 = 43,
  NAL_UNIT_RESERVED_NVCL44 = 44,
  NAL_UNIT_RESERVED_NVCL45 = 45,
  NAL_UNIT_RESERVED_NVCL46 = 46,
  NAL_UNIT_RESERVED_NVCL47 = 47,
};


int findH26xNalu(const unsigned char* p_pszData, int p_nDataLen, unsigned char* p_pszNaluType, int *nNalLen)
{
  int nStartCodePos = 0;
  int nFindNalType = 0;

  while (nStartCodePos < (p_nDataLen - 4))
  {
      if (p_pszData[nStartCodePos] == 0x00 && p_pszData[nStartCodePos + 1] == 0x00
          && p_pszData[nStartCodePos + 2] == 0x00 && p_pszData[nStartCodePos + 3] == 0x01) {
          if (nFindNalType) {
              *nNalLen = nStartCodePos;
              return nStartCodePos;
          }
          nFindNalType = 1;
          *p_pszNaluType = p_pszData[nStartCodePos + 4];
          nStartCodePos += 4;
      } else if (p_pszData[nStartCodePos] == 0x00 && p_pszData[nStartCodePos + 1] == 0x00
          && p_pszData[nStartCodePos + 2] == 0x01) {
          if (nFindNalType) {
              *nNalLen = nStartCodePos;
              return nStartCodePos;
          }
          nFindNalType = 1;
          *p_pszNaluType = p_pszData[nStartCodePos + 3];
          nStartCodePos += 3;
      }
      nStartCodePos++;
  }
  if (nFindNalType) {
      *nNalLen = p_nDataLen;
      return p_nDataLen;
  }
  return -1;
}

int findSPSPPSVPS(CodecImgFormat p_nStreamType, const unsigned char* p_pszFrameData, int p_nDataLen, int* p_pnSPS,
    int* p_pnPPS, int* p_pnVPS, int* p_pnSPSLen, int* p_pnPPSLen, int* p_pnVPSLen)
{
  int nNaluLen[6] = { 0 };  // 包含开始码长度 , 4
  int nStartCodePos[6] = { 0 };
  unsigned char nNaluType[6] = { 0 };
  int nNaluNum = 0;
  const unsigned char *pCheckMem = p_pszFrameData;
  int nLeftData = p_nDataLen;
  int nNalLen = 0;
  do {
      if (nNaluNum > 5) {
          RCLCPP_ERROR(rclcpp::get_logger("hobot_decoder_common"), "[findSPSPPSVPS]->nalNum=%d - err data.\n",
              nNaluNum);
          return -1;
      }
      nNalLen = findH26xNalu(pCheckMem, nLeftData,
          &nNaluType[nNaluNum], &nNaluLen[nNaluNum]);
      if (nNalLen < 0) {
        break;
      }
      nStartCodePos[nNaluNum] = (pCheckMem - p_pszFrameData);
      pCheckMem += nNaluLen[nNaluNum];
      nLeftData -= nNaluLen[nNaluNum];
      RCLCPP_INFO(rclcpp::get_logger("hobot_decoder_common"),
        "[findSPSPPSVPS]->type=%d, naLen=%d, start=%d, nalNum=%d",
        nNaluType[nNaluNum], nNaluLen[nNaluNum], nStartCodePos[nNaluNum], nNaluNum);
      ++nNaluNum;
  } while (1);

  // 根据nNaluType来输出
  for (int i = 0; i < nNaluNum; i++) {
    RCLCPP_INFO(rclcpp::get_logger("hobot_decoder_common"),
      "[findSPSPPSVPS] input nNaluType[%d]: %x", i, nNaluType[i]);
  
    if (CodecImgFormat::FORMAT_H264 == p_nStreamType)
        nNaluType[i] = nNaluType[i] & H264_GET_NALU_TYPE;
    else if (CodecImgFormat::FORMAT_H265 == p_nStreamType)
        nNaluType[i] = (nNaluType[i] & H265_GET_NALU_TYPE) >> 1;

    RCLCPP_INFO(rclcpp::get_logger("hobot_decoder_common"),
      "[findSPSPPSVPS] after convert nNaluType[%d]: %x", i, nNaluType[i]);
  
    switch (nNaluType[i]) {
      // todo 2023/1/10 帧类型的判断方法应该参考官方给出的方法
      // H264：https://github.com/cisco/openh264/blob/master/codec/api/wels/codec_def.h
      // H265：https://github.com/strukturag/libde265/blob/master/libde265/nal.h
      case static_cast<int>(NalUnitTypeH264::NAL_SPS):
      case static_cast<int>(NalUnitTypeH265::NAL_UNIT_SPS_NUT):
          *p_pnSPS = nStartCodePos[i];
          *p_pnSPSLen = nNaluLen[i];
          break;  // SPS
      case static_cast<int>(NalUnitTypeH264::NAL_PPS):
      case static_cast<int>(NalUnitTypeH265::NAL_UNIT_PPS_NUT):
          *p_pnPPS = nStartCodePos[i];
          *p_pnPPSLen = nNaluLen[i];
          break;  // PPS
      case static_cast<int>(NalUnitTypeH264::NAL_SEI):
      case static_cast<int>(NalUnitTypeH265::NAL_UNIT_PREFIX_SEI_NUT):
      case static_cast<int>(NalUnitTypeH265::NAL_UNIT_SUFFIX_SEI_NUT):
          break;  // SEI
      case static_cast<int>(NalUnitTypeH265::NAL_UNIT_VPS_NUT):
          *p_pnVPS = nStartCodePos[i];
          *p_pnVPSLen = nNaluLen[i];
          break;  // VPS
      case static_cast<int>(NalUnitTypeH264::NAL_SLICE_IDR):
      case static_cast<int>(NalUnitTypeH265::NAL_UNIT_IDR_W_RADL):
      case static_cast<int>(NalUnitTypeH265::NAL_UNIT_IDR_N_LP):
          // 查找到I帧
          return nStartCodePos[i];  // success
          break;  // I
    }
  }
  return -1;
}


}

#endif  // INCLUDE_HOBOT_DECODEC_COMMON_H_
