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

#ifndef INCLUDE_VIDEO_UTILS_HPP_
#define INCLUDE_VIDEO_UTILS_HPP_

#ifndef PLATFORM_X86

#include <arm_neon.h>
#include <sys/ioctl.h>
#include <cstring>
#include <sstream>

#else

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"

#include <immintrin.h>
#endif

namespace video_utils
{
#ifndef PLATFORM_X86

const uint8_t Y_SUBS[8] = { 16, 16, 16, 16, 16, 16, 16, 16 };
const uint8_t UV_SUBS[8] = { 128, 128, 128, 128, 128, 128, 128, 128 };
inline void NV12_TO_BGR24(unsigned char *_src, unsigned char* pUv, unsigned char *_RGBOut, int width, int height) {
  unsigned char *src = (unsigned char*)_src;
  unsigned char *RGBOut = (unsigned char*)_RGBOut;

  int i, j;
  int nWH = width * height;
  unsigned char *pY1 = src;
  unsigned char *pY2 = src + width;
  unsigned char *pUV = NULL;
  if (pUv)
    pUV = pUv;
  else
    pUV = src + nWH;

  uint8x8_t Y_SUBvec = vld1_u8(Y_SUBS);
  uint8x8_t UV_SUBvec = vld1_u8(UV_SUBS);

  // int width2 = width >> 1;
  int width3 = (width << 2) - width;
  unsigned char *RGBOut1 = RGBOut;
  unsigned char *RGBOut2 = RGBOut1 + width3;
  // unsigned char *RGBOut1 = RGBOut + 3 * width * (height - 2);
  // unsigned char *RGBOut2 = RGBOut1 + width3;

  unsigned char tempUV[8];
  // YUV 4:2:0
  // #pragma omp parallel for num_threads(4)
  for (j = 0; j < height; j += 2) {
      for (i = 0; i < width; i += 8) {
          tempUV[0] = pUV[1];
          tempUV[1] = pUV[3];
          tempUV[2] = pUV[5];
          tempUV[3] = pUV[7];

          tempUV[4] = pUV[0];
          tempUV[5] = pUV[2];
          tempUV[6] = pUV[4];
          tempUV[7] = pUV[6];

          pUV += 8;
          uint8x8_t nUVvec = vld1_u8(tempUV);
          int16x8_t nUVvec16 = vmovl_s8((int8x8_t)vsub_u8(nUVvec, UV_SUBvec));  // 减后区间-128到127
          int16x4_t V_4 = vget_low_s16((int16x8_t)nUVvec16);
          int16x4x2_t V16x4x2 = vzip_s16(V_4, V_4);
          // int16x8_t V16x8_;
          // memcpy(&V16x8_, &V16x4x2, 16);
          // int16x8_t* V16x8 = (int16x8_t*)(&V16x8_);
          int16x8_t* V16x8 = reinterpret_cast<int16x8_t*>(&V16x4x2);
          int16x4_t U_4 = vget_high_s16(nUVvec16);
          int16x4x2_t U16x4x2 = vzip_s16(U_4, U_4);
          int16x8_t* U16x8 = reinterpret_cast<int16x8_t*>(&U16x4x2);

          // 公式1
          int16x8_t VV1 = vmulq_n_s16(*V16x8, 102);
          int16x8_t UU1 = vmulq_n_s16(*U16x8, 129);
          int16x8_t VVUU1 = vmlaq_n_s16(vmulq_n_s16(*V16x8, 52), *U16x8, 25);

          uint8x8_t nYvec;
          uint8x8x3_t RGB;
          uint16x8_t Y16;
          // 上行
          nYvec = vld1_u8(pY1);
          pY1 += 8;
          // 公式1
          Y16 = vmulq_n_u16(vmovl_u8(vqsub_u8(nYvec, Y_SUBvec)), 74);  // 公式1

          RGB.val[0] = vqmovun_s16(vshrq_n_s16((int16x8_t)vaddq_u16(Y16, (uint16x8_t)UU1), 6));
          RGB.val[1] = vqmovun_s16(vshrq_n_s16((int16x8_t)vsubq_u16(Y16, (uint16x8_t)VVUU1), 6));
          RGB.val[2] = vqmovun_s16(vshrq_n_s16((int16x8_t)vaddq_u16(Y16, (uint16x8_t)VV1), 6));
          vst3_u8(RGBOut1, RGB);
          RGBOut1 += 24;

          // 下行
          nYvec = vld1_u8(pY2);
          pY2 += 8;
          // 公式1
          Y16 = vmulq_n_u16(vmovl_u8(vqsub_u8(nYvec, Y_SUBvec)), 74);  // 公式1
          RGB.val[0] = vqmovun_s16(vshrq_n_s16((int16x8_t)vaddq_u16(Y16, (uint16x8_t)UU1), 6));
          RGB.val[1] = vqmovun_s16(vshrq_n_s16((int16x8_t)vsubq_u16(Y16, (uint16x8_t)VVUU1), 6));
          RGB.val[2] = vqmovun_s16(vshrq_n_s16((int16x8_t)vaddq_u16(Y16, (uint16x8_t)VV1), 6));
          vst3_u8(RGBOut2, RGB);
          RGBOut2 += 24;
      }
      RGBOut1 += width3;
      RGBOut2 += width3;
      // RGBOut1 -= width9;
      // RGBOut2 -= width9;
      pY1 += width;
      pY2 += width;
  }
}

inline void NV12_TO_RGB24(unsigned char *_src, unsigned char* pUv, unsigned char *_RGBOut, int width, int height) {
  unsigned char *src = (unsigned char*)_src;
  unsigned char *RGBOut = (unsigned char*)_RGBOut;

  int i, j;
  int nWH = width * height;
  unsigned char *pY1 = src;
  unsigned char *pY2 = src + width;
  unsigned char *pUV = NULL;
  if (pUv)
    pUV = pUv;
  else
    pUV = src + nWH;

  uint8x8_t Y_SUBvec = vld1_u8(Y_SUBS);
  uint8x8_t UV_SUBvec = vld1_u8(UV_SUBS);

  int width3 = (width << 2) - width;
  unsigned char *RGBOut1 = RGBOut;
  unsigned char *RGBOut2 = RGBOut1 + width3;
  unsigned char tempUV[8];
  // YUV 4:2:0
  // #pragma omp parallel for num_threads(4)
  for (j = 0; j < height; j += 2) {
      for (i = 0; i < width; i += 8) {
          tempUV[0] = pUV[1];
          tempUV[1] = pUV[3];
          tempUV[2] = pUV[5];
          tempUV[3] = pUV[7];

          tempUV[4] = pUV[0];
          tempUV[5] = pUV[2];
          tempUV[6] = pUV[4];
          tempUV[7] = pUV[6];

          pUV += 8;
          uint8x8_t nUVvec = vld1_u8(tempUV);
          int16x8_t nUVvec16 = vmovl_s8((int8x8_t)vsub_u8(nUVvec, UV_SUBvec));  // 减后区间-128到127
          int16x4_t V_4 = vget_low_s16((int16x8_t)nUVvec16);
          int16x4x2_t V16x4x2 = vzip_s16(V_4, V_4);

          int16x8_t* V16x8 = reinterpret_cast<int16x8_t*>(&V16x4x2);
          int16x4_t U_4 = vget_high_s16(nUVvec16);
          int16x4x2_t U16x4x2 = vzip_s16(U_4, U_4);
          int16x8_t* U16x8 = reinterpret_cast<int16x8_t*>(&U16x4x2);

          // 公式1
          int16x8_t VV1 = vmulq_n_s16(*V16x8, 102);
          int16x8_t UU1 = vmulq_n_s16(*U16x8, 129);
          int16x8_t VVUU1 = vmlaq_n_s16(vmulq_n_s16(*V16x8, 52), *U16x8, 25);

          uint8x8_t nYvec;
          uint8x8x3_t RGB;
          uint16x8_t Y16;
          // 上行
          nYvec = vld1_u8(pY1);
          pY1 += 8;
          // 公式1
          Y16 = vmulq_n_u16(vmovl_u8(vqsub_u8(nYvec, Y_SUBvec)), 74);  // 公式1

          RGB.val[2] = vqmovun_s16(vshrq_n_s16((int16x8_t)vaddq_u16(Y16, (uint16x8_t)UU1), 6));
          RGB.val[1] = vqmovun_s16(vshrq_n_s16((int16x8_t)vsubq_u16(Y16, (uint16x8_t)VVUU1), 6));
          RGB.val[0] = vqmovun_s16(vshrq_n_s16((int16x8_t)vaddq_u16(Y16, (uint16x8_t)VV1), 6));
          vst3_u8(RGBOut1, RGB);
          RGBOut1 += 24;

          // 下行
          nYvec = vld1_u8(pY2);
          pY2 += 8;
          // 公式1
          Y16 = vmulq_n_u16(vmovl_u8(vqsub_u8(nYvec, Y_SUBvec)), 74);  // 公式1
          RGB.val[2] = vqmovun_s16(vshrq_n_s16((int16x8_t)vaddq_u16(Y16, (uint16x8_t)UU1), 6));
          RGB.val[1] = vqmovun_s16(vshrq_n_s16((int16x8_t)vsubq_u16(Y16, (uint16x8_t)VVUU1), 6));
          RGB.val[0] = vqmovun_s16(vshrq_n_s16((int16x8_t)vaddq_u16(Y16, (uint16x8_t)VV1), 6));
          vst3_u8(RGBOut2, RGB);
          RGBOut2 += 24;
      }
      RGBOut1 += width3;
      RGBOut2 += width3;
      pY1 += width;
      pY2 += width;
  }
}

inline void RGB24_to_NV12(const unsigned char* pRGB, unsigned char* pNV12, int width, int height)
{
    const uint8x8_t u8_zero = vdup_n_u8(0);
    const uint8x8_t u8_16 = vdup_n_u8(16);
    const uint16x8_t u16_rounding = vdupq_n_u16(128);

    const int16x8_t s16_zero = vdupq_n_s16(0);
    const int8x8_t s8_rounding = vdup_n_s8(-128);
    const int16x8_t s16_rounding = vdupq_n_s16(128);

    unsigned char* UVPtr = pNV12 + width * height;
    int pitch = width >> 4;

    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < pitch; ++i) {
            // Load RGB 16 pixel
            uint8x16x3_t pixel_rgb = vld3q_u8(pRGB);

            uint8x8_t high_r = vget_high_u8(pixel_rgb.val[0]);
            uint8x8_t low_r = vget_low_u8(pixel_rgb.val[0]);
            uint8x8_t high_g = vget_high_u8(pixel_rgb.val[1]);
            uint8x8_t low_g = vget_low_u8(pixel_rgb.val[1]);
            uint8x8_t high_b = vget_high_u8(pixel_rgb.val[2]);
            uint8x8_t low_b = vget_low_u8(pixel_rgb.val[2]);

            // NOTE:
            // declaration may not appear after executable statement in block
            uint16x8_t high_y;
            uint16x8_t low_y;

            uint8x8_t scalar = vdup_n_u8(66);  // scalar = 66
            high_y = vmull_u8(high_r, scalar);  // Y = R * 66
            low_y = vmull_u8(low_r, scalar);

            scalar = vdup_n_u8(129);
            high_y = vmlal_u8(high_y, high_g, scalar);  // Y = Y + R*129
            low_y = vmlal_u8(low_y, low_g, scalar);

            scalar = vdup_n_u8(25);
            high_y = vmlal_u8(high_y, high_b, scalar);  // Y = Y + B*25
            low_y = vmlal_u8(low_y, low_b, scalar);

            high_y = vaddq_u16(high_y, u16_rounding);  // Y = Y + 128
            low_y = vaddq_u16(low_y, u16_rounding);

            uint8x8_t u8_low_y = vshrn_n_u16(low_y, 8);  // Y = Y >> 8
            uint8x8_t u8_high_y = vshrn_n_u16(high_y, 8);

            low_y = vaddl_u8(u8_low_y, u8_16);  // Y = Y + 16
            high_y = vaddl_u8(u8_high_y, u8_16);

            uint8x16_t pixel_y = vcombine_u8(vqmovn_u16(low_y), vqmovn_u16(high_y));

            // Store
            vst1q_u8(pNV12, pixel_y);
            if (j % 2 == 0)
            {
                uint8x8x2_t mix_r = vuzp_u8(low_r, high_r);
                uint8x8x2_t mix_g = vuzp_u8(low_g, high_g);
                uint8x8x2_t mix_b = vuzp_u8(low_b, high_b);

                int16x8_t signed_r = vreinterpretq_s16_u16(vaddl_u8(mix_r.val[0], u8_zero));
                int16x8_t signed_g = vreinterpretq_s16_u16(vaddl_u8(mix_g.val[0], u8_zero));
                int16x8_t signed_b = vreinterpretq_s16_u16(vaddl_u8(mix_b.val[0], u8_zero));

                int16x8_t signed_u;
                int16x8_t signed_v;

                int16x8_t signed_scalar = vdupq_n_s16(-38);
                signed_u = vmulq_s16(signed_r, signed_scalar);

                signed_scalar = vdupq_n_s16(112);
                signed_v = vmulq_s16(signed_r, signed_scalar);

                signed_scalar = vdupq_n_s16(-74);
                signed_u = vmlaq_s16(signed_u, signed_g, signed_scalar);

                signed_scalar = vdupq_n_s16(-94);
                signed_v = vmlaq_s16(signed_v, signed_g, signed_scalar);

                signed_scalar = vdupq_n_s16(112);
                signed_u = vmlaq_s16(signed_u, signed_b, signed_scalar);

                signed_scalar = vdupq_n_s16(-18);
                signed_v = vmlaq_s16(signed_v, signed_b, signed_scalar);

                signed_u = vaddq_s16(signed_u, s16_rounding);
                signed_v = vaddq_s16(signed_v, s16_rounding);

                int8x8_t s8_u = vshrn_n_s16(signed_u, 8);
                int8x8_t s8_v = vshrn_n_s16(signed_v, 8);

                signed_u = vsubl_s8(s8_u, s8_rounding);
                signed_v = vsubl_s8(s8_v, s8_rounding);

                signed_u = vmaxq_s16(signed_u, s16_zero);
                signed_v = vmaxq_s16(signed_v, s16_zero);

                uint16x8_t unsigned_u = vreinterpretq_u16_s16(signed_u);
                uint16x8_t unsigned_v = vreinterpretq_u16_s16(signed_v);

                uint8x8x2_t result;
                result.val[0] = vqmovn_u16(unsigned_u);
                result.val[1] = vqmovn_u16(unsigned_v);

                vst2_u8(UVPtr, result);
                UVPtr += 16;
            }

            pRGB += 3 * 16;
            pNV12 += 16;
        }
    }
}

inline void BGR24_to_NV12(const unsigned char* pRGB, unsigned char* pNV12, int width, int height)
{
    const uint8x8_t u8_zero = vdup_n_u8(0);
    const uint8x8_t u8_16 = vdup_n_u8(16);
    const uint16x8_t u16_rounding = vdupq_n_u16(128);

    const int16x8_t s16_zero = vdupq_n_s16(0);
    const int8x8_t s8_rounding = vdup_n_s8(-128);
    const int16x8_t s16_rounding = vdupq_n_s16(128);

    unsigned char* UVPtr = pNV12 + width * height;
    int pitch = width >> 4;

    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < pitch; ++i) {
            // Load RGB 16 pixel
            uint8x16x3_t pixel_rgb = vld3q_u8(pRGB);

            uint8x8_t high_r = vget_high_u8(pixel_rgb.val[2]);
            uint8x8_t low_r = vget_low_u8(pixel_rgb.val[2]);
            uint8x8_t high_g = vget_high_u8(pixel_rgb.val[1]);
            uint8x8_t low_g = vget_low_u8(pixel_rgb.val[1]);
            uint8x8_t high_b = vget_high_u8(pixel_rgb.val[0]);
            uint8x8_t low_b = vget_low_u8(pixel_rgb.val[0]);

            // NOTE:
            // declaration may not appear after executable statement in block
            uint16x8_t high_y;
            uint16x8_t low_y;

            uint8x8_t scalar = vdup_n_u8(66);  // scalar = 66
            high_y = vmull_u8(high_r, scalar);  // Y = R * 66
            low_y = vmull_u8(low_r, scalar);

            scalar = vdup_n_u8(129);
            high_y = vmlal_u8(high_y, high_g, scalar);  // Y = Y + R*129
            low_y = vmlal_u8(low_y, low_g, scalar);

            scalar = vdup_n_u8(25);
            high_y = vmlal_u8(high_y, high_b, scalar);  // Y = Y + B*25
            low_y = vmlal_u8(low_y, low_b, scalar);

            high_y = vaddq_u16(high_y, u16_rounding);  // Y = Y + 128
            low_y = vaddq_u16(low_y, u16_rounding);

            uint8x8_t u8_low_y = vshrn_n_u16(low_y, 8);  // Y = Y >> 8
            uint8x8_t u8_high_y = vshrn_n_u16(high_y, 8);

            low_y = vaddl_u8(u8_low_y, u8_16);  // Y = Y + 16
            high_y = vaddl_u8(u8_high_y, u8_16);

            uint8x16_t pixel_y = vcombine_u8(vqmovn_u16(low_y), vqmovn_u16(high_y));

            // Store
            vst1q_u8(pNV12, pixel_y);
            if (j % 2 == 0)
            {
                uint8x8x2_t mix_r = vuzp_u8(low_r, high_r);
                uint8x8x2_t mix_g = vuzp_u8(low_g, high_g);
                uint8x8x2_t mix_b = vuzp_u8(low_b, high_b);

                int16x8_t signed_r = vreinterpretq_s16_u16(vaddl_u8(mix_r.val[0], u8_zero));
                int16x8_t signed_g = vreinterpretq_s16_u16(vaddl_u8(mix_g.val[0], u8_zero));
                int16x8_t signed_b = vreinterpretq_s16_u16(vaddl_u8(mix_b.val[0], u8_zero));

                int16x8_t signed_u;
                int16x8_t signed_v;

                int16x8_t signed_scalar = vdupq_n_s16(-38);
                signed_u = vmulq_s16(signed_r, signed_scalar);

                signed_scalar = vdupq_n_s16(112);
                signed_v = vmulq_s16(signed_r, signed_scalar);

                signed_scalar = vdupq_n_s16(-74);
                signed_u = vmlaq_s16(signed_u, signed_g, signed_scalar);

                signed_scalar = vdupq_n_s16(-94);
                signed_v = vmlaq_s16(signed_v, signed_g, signed_scalar);

                signed_scalar = vdupq_n_s16(112);
                signed_u = vmlaq_s16(signed_u, signed_b, signed_scalar);

                signed_scalar = vdupq_n_s16(-18);
                signed_v = vmlaq_s16(signed_v, signed_b, signed_scalar);

                signed_u = vaddq_s16(signed_u, s16_rounding);
                signed_v = vaddq_s16(signed_v, s16_rounding);

                int8x8_t s8_u = vshrn_n_s16(signed_u, 8);
                int8x8_t s8_v = vshrn_n_s16(signed_v, 8);

                signed_u = vsubl_s8(s8_u, s8_rounding);
                signed_v = vsubl_s8(s8_v, s8_rounding);

                signed_u = vmaxq_s16(signed_u, s16_zero);
                signed_v = vmaxq_s16(signed_v, s16_zero);

                uint16x8_t unsigned_u = vreinterpretq_u16_s16(signed_u);
                uint16x8_t unsigned_v = vreinterpretq_u16_s16(signed_v);

                uint8x8x2_t result;
                result.val[0] = vqmovn_u16(unsigned_u);
                result.val[1] = vqmovn_u16(unsigned_v);

                vst2_u8(UVPtr, result);
                UVPtr += 16;
            }

            pRGB += 3 * 16;
            pNV12 += 16;
        }
    }
}

#else
inline void NV12_to_BGR24(const uint8_t *DataIn, uint8_t *DataOut, int width, int height) {
  cv::Mat src(height * 3 / 2, width, CV_8UC1, (void*)DataIn);
  cv::Mat bgr_mat;
  cv::cvtColor(src, bgr_mat, CV_YUV2BGR_NV12);
  memcpy(DataOut, bgr_mat.ptr<uint8_t>(), height * width * 3);
}

inline void RGB24_to_BGR24(const uint8_t *DataIn, uint8_t *DataOut, int width, int height) {
  cv::Mat src(height , width, CV_8UC3, (void*)DataIn);
  cv::Mat bgr_mat;
  cv::cvtColor(src, bgr_mat, CV_RGB2BGR);
  memcpy(DataOut, bgr_mat.ptr<uint8_t>(), height * width * 3);
}

inline void BGR24_to_RGB24(const uint8_t *DataIn, uint8_t *DataOut, int width, int height) {
  cv::Mat src(height , width, CV_8UC3, (void*)DataIn);
  cv::Mat rgb_mat;
  cv::cvtColor(src, rgb_mat, CV_BGR2RGB);
  memcpy(DataOut, rgb_mat.ptr<uint8_t>(), height * width * 3);
}

inline void BGR24_to_NV12(const uint8_t *DataIn, uint8_t *DataOut, int width, int height)
{
  cv::Mat img_nv12;
  cv::Mat src(height , width, CV_8UC3, (void*)DataIn);
  if (height % 2 || width % 2) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_codec"),
          "[%s]: Image height and width must aligned by 2\n"
          "height: %d \nwidth: %d", __func__, height, width);
  }
  cv::Mat yuv_mat(height * 3 / 2, width, CV_8UC1);
  cv::cvtColor(src, yuv_mat, CV_BGR2YUV_I420);

  int32_t y_size = height * width;
  int32_t uv_height = height / 2;
  int32_t uv_width = width / 2;

  img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
  auto nv12_ptr = img_nv12.ptr<uint8_t>();
  if (yuv_mat.data == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("hobot_codec"),
      "[%s]: yuv_mat.data is null pointer", __func__);
  }
  auto yuv_ptr = yuv_mat.ptr<uint8_t>();
  memcpy(nv12_ptr, yuv_ptr, y_size);
  uint8_t *nv12_uv = nv12_ptr + y_size;
  uint8_t *u_data = yuv_ptr + y_size;
  uint8_t *v_data = u_data + uv_height * uv_width;
  for (int32_t i = 0; i < uv_width * uv_height; i++) {
    *nv12_uv++ = *u_data++;
    *nv12_uv++ = *v_data++;
  }
  memcpy(DataOut, img_nv12.ptr<uint8_t>(), height * width * 3 / 2);
}
#endif
}  // namespace video_utils
#endif  // INCLUDE_VIDEO_UTILS_HPP_
