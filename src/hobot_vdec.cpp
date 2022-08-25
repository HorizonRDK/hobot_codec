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

#include "include/hobot_vdec.h"
// H264 H265 MJPEG
#include "rclcpp/rclcpp.hpp"


HobotVdec::HobotVdec(int channel, const char *type) : HWCodec(channel, type), m_bFirstDec(1)
{
}
HobotVdec::~HobotVdec()
{
}

int HobotVdec::InitCodec() {
    HWCodec::InitCodec();
    return 0;
}
int HobotVdec::UninitCodec()
{
    return 0;
}

int HobotVdec::child_stop()
{
    int s32Ret;
    if (enCT_STOP == m_nCodecSt)
        return 0;
    m_nCodecSt = enCT_STOP;
    for (int i = 0; i < m_nMMZCnt; i++) {
        s32Ret = HB_SYS_Free(m_arrMMZ_PAddr[i], m_arrMMZ_VAddr[i]);
        if (s32Ret == 0) {
            RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "mmzFree paddr = 0x%x, vaddr = 0x%x i = %d \n", m_arrMMZ_PAddr[i],
                    m_arrMMZ_VAddr[i], i);
        }
    }
    s32Ret = HB_VP_Exit();
    if (s32Ret == 0) {
        RCLCPP_WARN(rclcpp::get_logger("HobotCodec"), "vp exit ok!\n");
    }

    s32Ret = HB_VDEC_StopRecvStream(m_nCodecChn);
    if (s32Ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"), "HB_VDEC_StopRecvStream failed\n");
        return -1;
    }
    HB_VDEC_ResetChn(m_nCodecChn);
    s32Ret = HB_VDEC_DestroyChn(m_nCodecChn);
    if (s32Ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"), "HB_VDEC_DestroyChn failed\n");
        return -1;
    }

    s32Ret = HB_VDEC_Module_Uninit();
    if (s32Ret) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"), "HB_VDEC_Module_Uninit: %d\n", s32Ret);
    }
    RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "Done\n");
    return 0;
}

int HobotVdec::chnAttr_init() {
    int streambufSize = 0;
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
    m_oVdecChnAttr.u32StreamBufSize = (m_nPicWidth * m_nPicHeight * 3 / 2 + 1024) & ~0x3ff;
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
    RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "[chnAttr_init]->type=%s, %d.\n", m_tsCodecType, m_enPalType);
    
    return 0;
}

int HobotVdec::init_vdec()
{
    int s32Ret;
    pthread_mutex_lock(&m_lckInit);
    // 初始化channel属性
    s32Ret = chnAttr_init();
    if (s32Ret) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"), "sample_venc_ChnAttr_init failded: %d\n", s32Ret);
    }
    // 创建channel
    s32Ret = HB_VDEC_CreateChn(m_nCodecChn, &m_oVdecChnAttr);
    if (s32Ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"), "HB_VDEC_CreateChn %d failed, %x.\n", m_nCodecChn, s32Ret);
        return -1;
    }
    // 设置channel属性
    s32Ret = HB_VDEC_SetChnAttr(m_nCodecChn, &m_oVdecChnAttr);  // config
    if (s32Ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"), "HB_VDEC_SetChnAttr failed\n");
        return -1;
    }
    VENC_RECV_PIC_PARAM_S pstRecvParam;
    pstRecvParam.s32RecvPicNum = 0;  // unchangable

    s32Ret = HB_VDEC_StartRecvStream(m_nCodecChn);
    if (s32Ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"), "HB_VDEC_StartRecvStream failed\n");
        return -1;
    }
    pthread_cond_signal(&m_condInit);
    pthread_mutex_unlock(&m_lckInit);

    int i = 0, error = 0;
    // 准备buffer
    for (i = 0; i < m_nMMZCnt; i++) {
        m_arrMMZ_VAddr[i] = NULL;
    }
    memset(m_arrMMZ_PAddr, 0, sizeof(m_arrMMZ_PAddr));
    mmz_size = m_nPicWidth * m_nPicHeight;
    for (i = 0; i < m_nMMZCnt; i++) {
        s32Ret = HB_SYS_Alloc(&m_arrMMZ_PAddr[i], reinterpret_cast<void **>(&m_arrMMZ_VAddr[i]), mmz_size);
        if (s32Ret == 0) {
            RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "mmzAlloc paddr = 0x%x, vaddr = 0x%x i = %d \n",
                    m_arrMMZ_PAddr[i], m_arrMMZ_VAddr[i], i);
        }
    }
    return 0;
}
int HobotVdec::child_start(int nPicWidth, int nPicHeight) {
    int s32Ret;
    int opt = 0;
    if (-1 == nPicWidth || -1 == nPicHeight)
        return 0;
    if (enCT_START == m_nCodecSt)
        return 0;
    m_nPicWidth = nPicWidth;
    m_nPicHeight = nPicHeight;
    // 初始化VP
    VP_CONFIG_S struVpConf;
    memset(&struVpConf, 0x00, sizeof(VP_CONFIG_S));
    struVpConf.u32MaxPoolCnt = 32;
    HB_VP_SetConfig(&struVpConf);
    s32Ret = HB_VP_Init();
    if (s32Ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"), "vp_init fail s32Ret = %d !\n", s32Ret);
    }
    // m_nCodecChn = 0;
    pthread_mutex_init(&m_lckInit, NULL);
    pthread_cond_init(&m_condInit, NULL);
    s32Ret = HB_VDEC_Module_Init();
    if (s32Ret) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"), "HB_VDEC_Module_Init: %d\n", s32Ret);
    }
    init_vdec();
    if (m_enPalType != PT_JPEG) {
        m_bFirstDec = 1;
    } else {
        m_bFirstDec = 0;
    }
    m_nCodecSt = enCT_START;
    RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "HB_VDEC_Module_Init: %d end.\n", s32Ret);
    return 0;
}
typedef enum
{
    H264_GET_NALU_TYPE  = 0x1F,   // 按位与提取NALU类型的掩码位
    H264_I_FRAME        = 0x05,
    H264_P_FRAME        = 0x01,
    H264_SPS_FRAME      = 0x07,
    H264_PPS_FRAME      = 0x08,
    H264_SEI_FRAME      = 0x06,
    H265_GET_NALU_TYPE  = 0x7E,   // 按位与提取NALU类型的掩码位
    H265_I_FRAME        = 0x26,
    H265_SS_FRAME       = 0x02,
    H265_VPS_FRAME      = 0x40,
    H265_SPS_FRAME      = 0x42,
    H265_PPS_FRAME      = 0x44,
}ENUM_NaluType;

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

int findSPSPPSVPS(int p_nStreamType, const unsigned char* p_pszFrameData, int p_nDataLen, int* p_pnSPS,
    int* p_pnPPS, int* p_pnVPS, int* p_pnSPSLen, int* p_pnPPSLen, int* p_pnVPSLen, int p_nStartCodeLen)
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
            RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"), "[findSPSPPSVPS]->nalNum=%d - err data.\n",
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
        // RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "[findSPSPPSVPS]->type=%d,naLen=%d,start=%d, nalNum=%d.\n",
        //   nNaluType[nNaluNum],nNaluLen[nNaluNum],nStartCodePos[nNaluNum], nNaluNum);
        ++nNaluNum;
    } while (1);
    // 根据nNaluType来输出
    for (int i = 0; i < nNaluNum; i++) {
        if (PT_H264 == p_nStreamType)
            nNaluType[i] = nNaluType[i] & H264_GET_NALU_TYPE;
        else if (PT_H265 == p_nStreamType)
            nNaluType[i] = nNaluType[i] & H265_GET_NALU_TYPE;

        switch (nNaluType[i]) {
        case H264_SPS_FRAME:
        case H265_SPS_FRAME:
            *p_pnSPS = nStartCodePos[i];
            *p_pnSPSLen = nNaluLen[i];
            break;  // SPS
        case H264_PPS_FRAME:
        case H265_PPS_FRAME:
            *p_pnPPS = nStartCodePos[i];
            *p_pnPPSLen = nNaluLen[i];
            break;  // PPS
        case 0x06:
            break;  // H264 SEI
        case H265_VPS_FRAME:
            *p_pnVPS = nStartCodePos[i];
            *p_pnVPSLen = nNaluLen[i];
            break;  // VPS
        case H264_I_FRAME:
        case H265_I_FRAME:
            return nStartCodePos[i];  // success
            break;  // I
        }
    }
    return -1;
}

// 如果 h264 数据，则有 头部处理逻辑
int HobotVdec::PutData(const uint8_t *pDataIn, int nLen, const struct timespec &time_stamp) {
    if (nLen > mmz_size) {
      std::string fname = std::to_string(time_stamp.tv_sec) + "_" + std::to_string(time_stamp.tv_nsec) + ".jpeg";
      RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"),
        "input nLen: %d exceeds alloc mmz_size: %d, dump to file: %s",
        nLen, mmz_size, fname.data());
    //   std::ofstream ofs(fname);
    //   ofs.write(reinterpret_cast<const char*>(pDataIn), nLen);
      return -1;
    }
    int s32Ret;
    if (enCT_START == m_nCodecSt) {
        // h26X 前几帧没有 I 帧，解码失败是正常的，只要不崩溃就可以
        if (m_enPalType != PT_JPEG) {
            if (m_bFirstDec) {
                // 检查是否是I 帧
                unsigned char *pszSPS = NULL, *pszPPS = NULL, *pszVPS = NULL;
                int nSPSPos = 0, nPPSPos = 0, nVPSPos = 0;
                int nSPSLen = 0, nPPSLen = 0, nVPSLen = 0;
                /*static int s_sTest = 0;
                if (0 == s_sTest) {
                    FILE *outFile = fopen("1.h26x", "wb");
                    fwrite(pDataIn, nLen, 1, outFile);
                    fclose(outFile);
                    s_sTest = 1;
                }*/
                int nIFramePos = findSPSPPSVPS(m_enPalType, pDataIn,
                    nLen, &nSPSPos, &nPPSPos, &nVPSPos, &nSPSLen, &nPPSLen, &nVPSLen, 4);
                if (nSPSLen <= 0) {
                    return -1;
                }
                m_bFirstDec = 0;
            }
        }
        VIDEO_FRAME_S pstFrame;
        VIDEO_STREAM_S pstStream;
        memset(&pstFrame, 0, sizeof(VIDEO_FRAME_S));
        memset(&pstStream, 0, sizeof(VIDEO_STREAM_S));
        m_nMMZidx = m_nUseCnt % m_nMMZCnt;
        VDEC_CHN_STATUS_S pstStatus;
        HB_VDEC_QueryStatus(m_nCodecChn, &pstStatus);
        if (pstStatus.cur_input_buf_cnt >= (uint32_t)m_nMMZCnt) {
            RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"), "[HB_VDEC_QueryStatus]->dlen=%d, inbufCnt=%d ->%d.\n",
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
        s32Ret = HB_VDEC_SendStream(m_nCodecChn, &pstStream, 3000);
        RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "[PutData] pts:%d, vir_ptr:%x, size:%d, ret=%d.\n",
                pstStream.pstPack.pts,
                pstStream.pstPack.vir_ptr,
                pstStream.pstPack.size, s32Ret);
        if (s32Ret == -HB_ERR_VDEC_OPERATION_NOT_ALLOWDED ||
                    s32Ret == -HB_ERR_VDEC_UNKNOWN) {
            return -2;
        }
        HWCodec::PutData(pDataIn, nLen, time_stamp);
        return 0;
    }
    /* else {
        // 如果是 h26x, 则解析 sps，得到 宽高，再进行解码
        unsigned char *pszSPS = NULL, *pszPPS = NULL, *pszVPS = NULL;
        int nSPSPos = 0, nPPSPos = 0, nVPSPos = 0;
        int nSPSLen = 0, nPPSLen = 0, nVPSLen = 0;
        int nIFramePos = findSPSPPSVPS(m_enPalType, pDataIn,
            nLen, &nSPSPos, &nPPSPos, &nVPSPos, &nSPSLen, &nPPSLen, &nVPSLen, nNaluTypePos);
    }*/
    return -100;
}

int HobotVdec::ReleaseFrame(TFrameData *pFrame)
{
    if (pFrame) {
        // VIDEO_FRAME_S curFrameInfo;
        // curFrameInfo.stVFrame.vir_ptr[0] = (hb_char*)pFrame->mPtrY;
        // curFrameInfo.stVFrame.vir_ptr[1] = (hb_char*)pFrame->mPtrUV;
        RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "[%s] 0x%x- 0x%x, %dx%d\n", __func__,
            pFrame->mPtrY, pFrame->mPtrUV, pFrame->mWidth, pFrame->mHeight);
        return HB_VDEC_ReleaseFrame(m_nCodecChn, &m_curFrameInfo);
    }
    return 0;
}
static int s_test = 1;
int HobotVdec::GetFrame(TFrameData *pOutFrm) {
    int s32Ret;
    if (enCT_START == m_nCodecSt && 0 == m_bFirstDec) {
        // VIDEO_FRAME_S curFrameInfo;
        int idx = 0;
        struct timeval now;
        struct timespec outtime;

        s32Ret = HB_VDEC_GetFrame(m_nCodecChn, &m_curFrameInfo, 1000);
        if (s32Ret == 0) {
            HWCodec::GetFrame(pOutFrm);
            // m_curFrameInfo.stVFrame.height * m_curFrameInfo.stVFrame.width
            pOutFrm->mPtrY = reinterpret_cast<uint8_t*>(m_curFrameInfo.stVFrame.vir_ptr[0]);
            pOutFrm->mPtrUV = reinterpret_cast<uint8_t*>(m_curFrameInfo.stVFrame.vir_ptr[1]);
            // m_curFrameInfo.stVFrame.height * m_curFrameInfo.stVFrame.width / 2
            pOutFrm->mDataLen = m_curFrameInfo.stVFrame.height * m_curFrameInfo.stVFrame.width * 3 / 2;
            pOutFrm->mWidth = m_curFrameInfo.stVFrame.width;
            pOutFrm->mHeight = m_curFrameInfo.stVFrame.height;
            pOutFrm->mFrameFmt = HB_PIXEL_FORMAT_NV12;  // 通通 nv12
            RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "[%s]->0x%x:0x%x 1-0x%x, w:h=%dx%d, dlen=%d.\n",
                __func__, pOutFrm->mPtrY, m_curFrameInfo.stVFrame.vir_ptr[0], m_curFrameInfo.stVFrame.vir_ptr[1],
                m_curFrameInfo.stVFrame.width, m_curFrameInfo.stVFrame.height, pOutFrm->mDataLen);
            if (0 == s_test) {
                FILE *outFile = fopen("decode.nv12", "wb");
                fwrite(pOutFrm->mPtrY, pOutFrm->mWidth * pOutFrm->mHeight, 1, outFile);
                fwrite(pOutFrm->mPtrUV, pOutFrm->mWidth * pOutFrm->mHeight / 2, 1, outFile);
                fclose(outFile);
                s_test = 1;
            }
            // HB_VDEC_ReleaseFrame(m_nCodecChn, &m_curFrameInfo);
            return 0;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"), "HB_VDEC_GetFrame failed:%d\n", s32Ret);
            return -1;
        }
    }
    return -100;
}

int HobotVdec::build_dec_seq_header(uint8_t * pbHeader, int* sizelength, uint8_t* pbMetaData, int nMetaDLen)
{
    uint8_t* p =    pbMetaData;
    uint8_t *a =    p + 4 - ((int64_t) p & 3);
    uint8_t* t =    pbHeader;
    int   size = 0;
    int   sps, pps, i, nal;

    *sizelength = 4;  // default size length(in bytes) = 4
    if (m_enPalType == PT_H264) {
        if (nMetaDLen > 1 && pbMetaData && pbMetaData[0] == 0x01) {
            // check mov/mo4 file format stream
            p += 4;
            *sizelength = (*p++ & 0x3) + 1;
            sps = (*p & 0x1f);  // Number of sps
            p++;
            for (i = 0; i < sps; i++) {
                nal = (*p << 8) + *(p + 1) + 2;
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x01);
                SET_BUFFER(t, p+2, nal-2);
                p += nal;
                size += (nal - 2 + 4);  // 4 => length of start code to be inserted
            }
            pps = *(p++);  // number of pps
            for (i = 0; i < pps; i++) {
                nal = (*p << 8) + *(p + 1) + 2;
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x00);
                SET_BYTE(t, 0x01);
                SET_BUFFER(t, p+2, nal-2);
                p += nal;
                size += (nal - 2 + 4);  // 4 => length of start code to be inserted
            }
        } else if (nMetaDLen > 3) {
            size = -1;  // return to meaning of invalid stream data;
            for (; p < a; p++) {
                if (p[0] == 0 && p[1] == 0 && p[2] == 1)  {
                    // find startcode
                    size = nMetaDLen;
                    if (!pbHeader || !pbMetaData)
                        return 0;
                    SET_BUFFER(pbHeader, pbMetaData, size);
                    break;
                }
            }
        }
    } else if (m_enPalType == PT_H265) {
        if (nMetaDLen > 1 && pbMetaData && pbMetaData[0] == 0x01) {
            static const int8_t nalu_header[4] = { 0, 0, 0, 1 };
            int numOfArrays = 0;
            uint16_t numNalus = 0;
            uint16_t nalUnitLength = 0;
            uint32_t offset = 0;

            p += 21;
            *sizelength = (*p++ & 0x3) + 1;
            numOfArrays = *p++;
            while (numOfArrays--) {
                p++;   // NAL type
                numNalus = (*p << 8) + *(p + 1);
                p+=2;
                for (i = 0; i < numNalus; i++) {
                    nalUnitLength = (*p << 8) + *(p + 1);
                    p += 2;
                    // if(i == 0)
                    {
                        memcpy(pbHeader + offset, nalu_header, 4);
                        offset += 4;
                        memcpy(pbHeader + offset, p, nalUnitLength);
                        offset += nalUnitLength;
                    }
                    p += nalUnitLength;
                }
            }
            size = offset;
        } else if (nMetaDLen > 3) {
            size = -1;  // return to meaning of invalid stream data;
            for (; p < a; p++) {
                if (p[0] == 0 && p[1] == 0 && p[2] == 1) {
                    // find startcode
                    size = nMetaDLen;
                    if (!pbHeader || !pbMetaData)
                        return 0;
                    SET_BUFFER(pbHeader, pbMetaData, size);
                    break;
                }
            }
        }
    } else {
        SET_BUFFER(pbHeader, pbMetaData, nMetaDLen);
        size = nMetaDLen;
    }

    return size;
}
