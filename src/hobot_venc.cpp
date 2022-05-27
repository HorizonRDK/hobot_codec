/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#include "include/hobot_venc.h"
// H264 H265 MJPEG
#include "include/video_utils.hpp"

HobotVenc::HobotVenc(int channel, const char *type) : HWCodec(channel, type),
    m_fJpgQuality(0), m_fEncQp(20.0)
{
}
HobotVenc::~HobotVenc() {
}

int HobotVenc::InitCodec()
{
    HWCodec::InitCodec();
    return 0;
}
int HobotVenc::UninitCodec()
{
    return 0;
}
int HobotVenc::child_stop()
{
    int s32Ret;
    if (enCT_STOP == m_nCodecSt)
        return 0;
    m_nCodecSt = enCT_STOP;

    for (int i = 0; i < m_nMMZCnt; i++) {
        s32Ret = HB_SYS_Free(m_arrMMZ_PAddr[i], m_arrMMZ_VAddr[i]);
        if (s32Ret == 0) {
            ROS_printf(2, "mmzFree paddr = 0x%x, vaddr = 0x%x i = %d \n", m_arrMMZ_PAddr[i],
                    m_arrMMZ_VAddr[i], i);
        }
    }
    s32Ret = HB_VENC_StopRecvFrame(m_nCodecChn);
    if (s32Ret != 0) {
        ROS_printf(0, "HB_VENC_StopRecvFrame failed\n");
        return -1;
    }
    s32Ret = HB_VENC_DestroyChn(m_nCodecChn);
    if (s32Ret != 0) {
        ROS_printf(0, "HB_VENC_DestroyChn failed\n");
        return -1;
    }
    s32Ret = HB_VP_Exit();
    if (s32Ret == 0) {
        ROS_printf(2, "vp exit ok!\n");
    }
    s32Ret = HB_VENC_Module_Uninit();
    if (s32Ret) {
        ROS_printf(0, "HB_VENC_Module_Uninit: %d\n", s32Ret);
    }
    return 0;
}

int HobotVenc::exec_init()
{
    int s32Ret;
    m_nCodecSt = 100;
    pthread_mutex_init(&m_lckInit, NULL);
    pthread_cond_init(&m_condInit, NULL);

    s32Ret = HB_VENC_Module_Init();
    ROS_printf(2, "===>HB_VENC_Module_Init: %d\n", s32Ret);
    // 初始化VP
    VP_CONFIG_S struVpConf;
    memset(&struVpConf, 0x00, sizeof(VP_CONFIG_S));
    struVpConf.u32MaxPoolCnt = 32;
    HB_VP_SetConfig(&struVpConf);
    s32Ret = HB_VP_Init();
    ROS_printf(2, "===>vp_init s32Ret = %d !\n", s32Ret);

    if (0 != init_venc()) {
        HB_VP_Exit();
        HB_VENC_Module_Uninit();
        abort();  // 直接退出
        return -1;
    }
    // Create(nPicWidth, nPicHeight, 8000);
    VENC_RECV_PIC_PARAM_S pstRecvParam;
    pstRecvParam.s32RecvPicNum = 0;  // unchangable

    s32Ret = HB_VENC_StartRecvFrame(m_nCodecChn, &pstRecvParam);
    ROS_printf(2, "HB_VENC_StartRecvFrame chn=%d,ret=%d.\n", m_nCodecChn, s32Ret);
    if (s32Ret != 0) {
        abort();  // 直接退出
        return -2;
    }

    // 准备buffer
    int i = 0, error = 0;
    for (i = 0; i < m_nMMZCnt; i++) {
        m_arrMMZ_VAddr[i] = NULL;
        m_arrMMZ_PAddr[i] = 0;
    }
    // memset(m_arrMMZ_PAddr, 0, sizeof(m_arrMMZ_PAddr));
    int mmz_size = m_nPicWidth * m_nPicHeight * 3 / 2;
    for (i = 0; i < m_nMMZCnt; i++) {
        // s32Ret = HB_SYS_Alloc(&m_arrMMZ_PAddr[i], reinterpret_cast<void **>(&m_arrMMZ_VAddr[i]), mmz_size);
        s32Ret = HB_SYS_Alloc(&m_arrMMZ_PAddr[i], reinterpret_cast<void **>(&m_arrMMZ_VAddr[i]), mmz_size);
        if (s32Ret == 0) {
            ROS_printf(2, "mmzAlloc w:h=%d:%d, paddr = 0x%x, vaddr = 0x%x i = %d \n",
                m_nPicWidth, m_nPicHeight, m_arrMMZ_PAddr[i], m_arrMMZ_VAddr[i], i);
        }
    }

    ROS_printf(2, "[%s]: %d end.\n", __func__, s32Ret);
    m_nCodecSt = enCT_START;
    return 0;
}

int HobotVenc::child_start(int nPicWidth, int nPicHeight) {
    int s32Ret;
    int opt = 0;
    if (enCT_START == m_nCodecSt) {
        // 增加鲁棒性测试，由于可能没设置 ROS_DOMAIN_ID，可能会收到不同分辨率，不同分辨率直接失败不处理
        if (m_nPicWidth != nPicWidth || m_nPicHeight != nPicHeight)
            return -1;
        return 0;
    }
    m_nPicWidth = nPicWidth;
    m_nPicHeight = nPicHeight;
// #ifdef THRD_INIT
    // if (100 != m_nCodecSt)
    //    m_spThrdInit = std::make_shared<std::thread>(std::bind(&HobotVenc::exec_init, this));
// #else
    return exec_init();
// #endif
}

void HobotVenc::SetCodecAttr(const char* tsName, float fVal)
{
    if (0 == strcmp("jpg_quality", tsName)) {
        m_fJpgQuality = fVal;
    } else if (0 == strcmp("enc_qp", tsName)) {
        m_fEncQp = fVal;
    }
}

int HobotVenc::init_venc()
{
    int s32Ret;
    int pts = 0;
    int count = 0;

    pthread_mutex_lock(&m_lckInit);
    // 初始化channel属性
    s32Ret = chnAttr_init();
    ROS_printf(2, "sample_venc_ChnAttr_init : %d\n", s32Ret);
    // 创建channel
    s32Ret = HB_VENC_CreateChn(m_nCodecChn, &m_oVencChnAttr);
    ROS_printf(2, "HB_VENC_CreateChn type=%d, %d , %d.\n", m_enPalType, m_nCodecChn, s32Ret);
    if (s32Ret != 0) {
        return -1;
    }
    // 配置Rc参数
    if (m_enPalType == PT_JPEG || m_enPalType == PT_MJPEG)
        s32Ret = venc_setRcParam(8000);
    else
        s32Ret = venc_setRcParam(3000);
    ROS_printf(2, "sample_venc_setRcParam ret: %d\n", s32Ret);

    // 设置channel属性
    s32Ret = HB_VENC_SetChnAttr(m_nCodecChn, &m_oVencChnAttr);  // config
    ROS_printf(2, "HB_VENC_SetChnAttr ret=%d.\n", s32Ret);
    if (s32Ret != 0) {
        HB_VENC_DestroyChn(m_nCodecChn);
        pthread_mutex_unlock(&m_lckInit);
        return -1;
    }
    pthread_cond_signal(&m_condInit);
    pthread_mutex_unlock(&m_lckInit);
    return 0;
}
// nv12
static int s_enctest = 0;

int HobotVenc::PutData(const uint8_t *pDataIn, int nLen, const struct timespec &time_stamp) {
    int s32Ret;
    if (enCT_START == m_nCodecSt) {
        // 先来获取编码
        // m_MtxLstFrames.lock();
        uint32_t tmNow = video_utils::GetTickCount();
        /*int tmSleep = 0;
        if (0 == m_tmLastPush) {
            m_tmLastPush = tmNow;
        } else {
            tmSleep = 20 + m_tmLastPush - tmNow;
        }
        if (tmSleep > 0) {
            usleep(tmSleep * 1000);
            ROS_printf(2, "[PutData]->chn=%d,w:h=%d:%d,len=%d, use=%d-get=%d,last=%d,now=%d sleep %d ms,begin\n",
                m_nCodecChn, m_nPicWidth, m_nPicHeight, nLen, m_nUseCnt, m_nGetCnt, m_tmLastPush, tmNow, tmSleep);
        }
        m_tmLastPush = tmNow;*/
        VIDEO_FRAME_S pstFrame = { 0 };  // 至关重要，有些变量要初始赋值为 0(false)
        int offset = m_nPicWidth * m_nPicHeight;
        m_nMMZidx = m_nUseCnt % m_nMMZCnt;

        memcpy(reinterpret_cast<void*>(m_arrMMZ_VAddr[m_nMMZidx]), pDataIn, nLen);
        pstFrame.stVFrame.width = m_nPicWidth;
        pstFrame.stVFrame.height = m_nPicHeight;
        pstFrame.stVFrame.size = nLen;  // m_nPicWidth * m_nPicHeight * 3 / 2;
        pstFrame.stVFrame.pix_format = HB_PIXEL_FORMAT_NV12;
        pstFrame.stVFrame.phy_ptr[0] = m_arrMMZ_PAddr[m_nMMZidx];
        pstFrame.stVFrame.phy_ptr[1] = m_arrMMZ_PAddr[m_nMMZidx] + offset;
        // pstFrame.stVFrame.phy_ptr[2] = m_arrMMZ_PAddr[m_nMMZidx] + offset * 5 / 4;
        pstFrame.stVFrame.vir_ptr[0] = m_arrMMZ_VAddr[m_nMMZidx];
        pstFrame.stVFrame.vir_ptr[1] = m_arrMMZ_VAddr[m_nMMZidx] + offset;
        // pstFrame.stVFrame.vir_ptr[2] = m_arrMMZ_VAddr[m_nMMZidx] + offset * 5 / 4;

        pstFrame.stVFrame.pts = tmNow;
        m_nUseCnt++;  // wuwlNG
        s32Ret = HB_VENC_SendFrame(m_nCodecChn, &pstFrame, 3000);
        /*uint32_t tmNowWrite = video_utils::GetTickCount();
        if (0 == s_enctest) {
            char fileName[128] = { 0 };
            snprintf(fileName, sizeof(fileName), "enc-put-%d.nv12", 0);  // m_nUseCnt);
            FILE *outFile = fopen(fileName, "wb");
            fwrite(pDataIn, nLen, 1, outFile);
            fclose(outFile);
        }
        ROS_printf(2, "[HB_VENC_SendFrame]->chn=%d,w:h=%d:%d,len=%d,idx=%d, use=%d,ret=%d ,write=%d ms.\n",
            m_nCodecChn, m_nPicWidth, m_nPicHeight, nLen, m_nMMZidx, m_nUseCnt, s32Ret,
            time_stamp.tv_sec, time_stamp.tv_nsec, video_utils::GetTickCount() - tmNowWrite);*/
        ROS_printf(2, "[HB_VENC_SendFrame]->chn=%d,w:h=%d:%d,len=%d,idx=%d, use=%d:%d ms,ret=%d tm=%d.%d.\n",
            m_nCodecChn, m_nPicWidth, m_nPicHeight, nLen, m_nMMZidx, m_nUseCnt,
            video_utils::GetTickCount() - tmNow, s32Ret, time_stamp.tv_sec, time_stamp.tv_nsec);
        if (s32Ret != 0) {
            return -1;
        }
        HWCodec::PutData(pDataIn, nLen, time_stamp);
        // usleep(50*1000);
        // 可能需要保存编码的结果，因为 扔进去的 太快了，扔之前，先读取一下，保存起来，给获取的使用。
        return 0;
    }
    return -100;
}

FILE *outH264File = NULL;  // fopen("enc.jpg", "wb");
int HobotVenc::ReleaseFrame(TFrameData *pFrame)
{
    if (pFrame) {
        // VIDEO_STREAM_S pstStreamInfo;
        // pstStreamInfo.pstPack.vir_ptr = (hb_char*)pFrame->mPtrData;
        int s32Ret = HB_VENC_ReleaseStream(m_nCodecChn, &m_curGetStream);
        m_curGetStream.pstPack.vir_ptr = NULL;
        m_curGetStream.pstPack.size = 0;
        if (s32Ret != 0)
            ROS_printf(0, "[%s]0x%x-%dx%d,ret=%d\n", __func__,
                pFrame->mPtrData, pFrame->mWidth, pFrame->mHeight, s32Ret);
        return s32Ret;
    }
    return 0;
}

int HobotVenc::GetFrame(TFrameData *pOutFrm) {
    int s32Ret;
    if (enCT_START == m_nCodecSt) {
        if (-1 == HWCodec::GetFrame(pOutFrm))
            return -1;
        s32Ret = HB_VENC_GetStream(m_nCodecChn, &m_curGetStream, 3000);  // m_curGetStream
        ROS_printf(2, "[HB_VENC_GetStream]->chn=%d w:h=%d:%d,getNum=%d,dlen=%d,ret=%d,tm=%d.%d\n",
            m_nCodecChn, m_nPicWidth, m_nPicHeight, m_nGetCnt, m_curGetStream.pstPack.size,
            s32Ret, pOutFrm->time_stamp.tv_sec, pOutFrm->time_stamp.tv_nsec);
        if (s32Ret != 0) {
            usleep(10000);
            return -1;
        }
        pOutFrm->mPtrData = reinterpret_cast<uint8_t*>(m_curGetStream.pstPack.vir_ptr);
        pOutFrm->mDataLen = m_curGetStream.pstPack.size;
        pOutFrm->mWidth = m_nPicWidth;
        pOutFrm->mHeight = m_nPicHeight;
        pOutFrm->mFrameFmt = m_enPalType;
        ++m_nGetCnt;
#ifdef TEST_SAVE
        if (NULL == outH264File)
            outH264File = fopen("enc.dat", "wb");
        if (outH264File) {
            fwrite(m_curGetStream.pstPack.vir_ptr,
                m_curGetStream.pstPack.size, 1, outH264File);
        }
#endif
        /*if (0 == s_enctest) {
            FILE *outFile = fopen("enc.jpg", "wb");
            fwrite(m_curGetStream.pstPack.vir_ptr,
                            m_curGetStream.pstPack.size, 1, outFile);
            fclose(outFile);
            s_enctest = 1;
        }*/
        // s32Ret = HB_VENC_ReleaseStream(m_nCodecChn, &m_curGetStream);
        return 0;
    }
    return -100;
}

int HobotVenc::chnAttr_init() {
    int streambufSize = 0;
    // 该步骤必不可少
    memset(&m_oVencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    // 设置编码模型分别为 PT_H264 PT_H265 PT_MJPEG
    m_oVencChnAttr.stVencAttr.enType = m_enPalType;
    // 设置编码分辨率
    m_oVencChnAttr.stVencAttr.u32PicWidth = m_nPicWidth;
    m_oVencChnAttr.stVencAttr.u32PicHeight = m_nPicHeight;
    // 设置像素格式 NV12格式
    m_oVencChnAttr.stVencAttr.enPixelFormat = HB_PIXEL_FORMAT_NV12;
    // 配置图像镜像属性 无镜像
    m_oVencChnAttr.stVencAttr.enMirrorFlip = DIRECTION_NONE;
    // 配置图像旋转属性 不旋转
    m_oVencChnAttr.stVencAttr.enRotation = CODEC_ROTATION_0;
    // 配置图像剪裁属性 不剪裁
    m_oVencChnAttr.stVencAttr.stCropCfg.bEnable = HB_FALSE;
    // 输入图像大小 1024对齐
    streambufSize = (m_nPicWidth * m_nPicHeight * 3 / 2 + 1024) & ~0x3ff;
    // vlc_buf_size为经验值，可以减少RAM使用，如果想使用默认值则保持为0
    if (m_nPicWidth * m_nPicHeight > 2688 * 1522) {
        m_oVencChnAttr.stVencAttr.vlc_buf_size = 7900 * 1024;
    } else if (m_nPicWidth * m_nPicHeight > 1920 * 1080) {
        m_oVencChnAttr.stVencAttr.vlc_buf_size = 4 * 1024 * 1024;
    } else if (m_nPicWidth * m_nPicHeight > 1280 * 720) {
        m_oVencChnAttr.stVencAttr.vlc_buf_size = 2100 * 1024;
    } else if (m_nPicWidth * m_nPicHeight > 704 * 576) {
        m_oVencChnAttr.stVencAttr.vlc_buf_size = 2100 * 1024;
    } else {
        m_oVencChnAttr.stVencAttr.vlc_buf_size = 2048 * 1024;
    }
    if (m_enPalType == PT_JPEG || m_enPalType == PT_MJPEG) {
        // 输出码流个数
        m_oVencChnAttr.stVencAttr.u32BitStreamBufferCount = 1;
        // 输入图像个数
        m_oVencChnAttr.stVencAttr.u32FrameBufferCount = 2;
        m_oVencChnAttr.stVencAttr.bExternalFreamBuffer = HB_TRUE;
        // 关闭DCF
        m_oVencChnAttr.stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
        // 质量系数，越小质量越好
        m_oVencChnAttr.stVencAttr.stAttrJpeg.quality_factor = m_fJpgQuality;  // 0;
        // 配置为0 不建议更改
        m_oVencChnAttr.stVencAttr.stAttrJpeg.restart_interval = 0;
        // 4096对齐
        m_oVencChnAttr.stVencAttr.u32BitStreamBufSize = (streambufSize + 4096) & ~4095;
    } else {
        // 输出码流个数
        m_oVencChnAttr.stVencAttr.u32BitStreamBufferCount = 3;
        // 输入图像个数
        m_oVencChnAttr.stVencAttr.u32FrameBufferCount = 3;
        m_oVencChnAttr.stVencAttr.bExternalFreamBuffer = HB_TRUE;
        m_oVencChnAttr.stVencAttr.u32BitStreamBufSize = streambufSize;
    }

    if (m_enPalType == PT_H265) {
        // 配置编码模式为H265VBR
        m_oVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
        // 不使能QpMap
        m_oVencChnAttr.stRcAttr.stH265Vbr.bQpMapEnable = HB_FALSE;
        // 设置I帧Qp值
        m_oVencChnAttr.stRcAttr.stH265Vbr.u32IntraQp = m_fEncQp;  // 20;
        // 设置I帧间隔
        m_oVencChnAttr.stRcAttr.stH265Vbr.u32IntraPeriod = 60;
        // 设置帧率
        m_oVencChnAttr.stRcAttr.stH265Vbr.u32FrameRate = 30;
    }
    if (m_enPalType == PT_H264) {
        // 配置编码模式为H264VBR
        m_oVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
        // 不使能QpMap
        m_oVencChnAttr.stRcAttr.stH264Vbr.bQpMapEnable = HB_FALSE;
        // 设置I帧Qp值
        m_oVencChnAttr.stRcAttr.stH264Vbr.u32IntraQp = 20;
        // 设置I帧间隔
        m_oVencChnAttr.stRcAttr.stH264Vbr.u32IntraPeriod = 60;
        // 设置帧率
        m_oVencChnAttr.stRcAttr.stH264Vbr.u32FrameRate = 30;
        // h264_profile为0，系统自动配置
        m_oVencChnAttr.stVencAttr.stAttrH264.h264_profile = (VENC_H264_PROFILE_E)0;
        // h264_level为0，系统自动配置
        m_oVencChnAttr.stVencAttr.stAttrH264.h264_level = (HB_H264_LEVEL_E)0;
    }
    // 设置GOP结构
    m_oVencChnAttr.stGopAttr.u32GopPresetIdx = 2;
    // 设置IDR帧类型
    m_oVencChnAttr.stGopAttr.s32DecodingRefreshType = 2;
    ROS_printf(2, "[%s]->rc=%d, vlcSz=%d, streamSz=%d, type=j-%d:4-%d:5-%d cur=%d.\n",
        __func__, m_oVencChnAttr.stRcAttr.enRcMode, m_oVencChnAttr.stVencAttr.vlc_buf_size,
        m_oVencChnAttr.stVencAttr.u32BitStreamBufSize, PT_JPEG, PT_H264, PT_H265, m_enPalType);
    return 0;
}

int HobotVenc::venc_setRcParam(int bitRate) {
    VENC_RC_ATTR_S *pstRcParam;
    int s32Ret;

    if (m_oVencChnAttr.stVencAttr.enType == PT_H264) {
        pstRcParam = &(m_oVencChnAttr.stRcAttr);
        // 为什么之前是VBR 这里改为CBR 之前的VBR是必须的吗？
        m_oVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        s32Ret = HB_VENC_GetRcParam(m_nCodecChn, pstRcParam);
        if (s32Ret != 0) {
            ROS_printf(0, "HB_VENC_GetRcParam failed.\n");
            return -1;
        }
        // 设置码率
        pstRcParam->stH264Cbr.u32BitRate = bitRate;
        // 设置帧率
        pstRcParam->stH264Cbr.u32FrameRate = 30;
        // 设置I帧间隔
        pstRcParam->stH264Cbr.u32IntraPeriod = 30;
        // 设置VbvBufferSize，与码率、解码器速率有关，一般在1000～5000
        pstRcParam->stH264Cbr.u32VbvBufferSize = 3000;
        ROS_printf(2, "[%s]->h264 enRcMode = %d, u32VbvBufferSize = %d, bitRate=%d mmmmmmmmmmmmmmmmmm   \n",
            __func__, m_oVencChnAttr.stRcAttr.enRcMode, m_oVencChnAttr.stRcAttr.stH264Cbr.u32VbvBufferSize, bitRate);
    } else if (m_oVencChnAttr.stVencAttr.enType == PT_H265) {
        pstRcParam = &(m_oVencChnAttr.stRcAttr);
        m_oVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
        s32Ret = HB_VENC_GetRcParam(m_nCodecChn, pstRcParam);
        if (s32Ret != 0) {
            ROS_printf(0, "HB_VENC_GetRcParam failed.\n");
            return -1;
        }
        // 设置码率
        pstRcParam->stH265Cbr.u32BitRate = bitRate;
        // 设置帧率
        pstRcParam->stH265Cbr.u32FrameRate = 30;
        // 设置I帧间隔
        pstRcParam->stH265Cbr.u32IntraPeriod = 30;
        // 设置VbvBufferSize，与码率、解码器速率有关，一般在1000～5000
        pstRcParam->stH265Cbr.u32VbvBufferSize = 3000;
        ROS_printf(0, "[%s]->h265 enRcMode = %d, u32VbvBufferSize = %d, bitRate=%d mmmmmmmmmmmmmmmmmm   \n",
            __func__, m_oVencChnAttr.stRcAttr.enRcMode, m_oVencChnAttr.stRcAttr.stH265Cbr.u32VbvBufferSize, bitRate);
    }
    return 0;
}

int HobotVenc::VencChnAttrInit(VENC_CHN_ATTR_S *pVencChnAttr, PAYLOAD_TYPE_E p_enType,
            int p_Width, int p_Height, PIXEL_FORMAT_E pixFmt) {
    int streambuf = 2*1024*1024;

    memset(pVencChnAttr, 0, sizeof(VENC_CHN_ATTR_S));
    pVencChnAttr->stVencAttr.enType = p_enType;

    pVencChnAttr->stVencAttr.u32PicWidth = p_Width;
    pVencChnAttr->stVencAttr.u32PicHeight = p_Height;
    // pVencChnAttr->stVencAttr.u32InputFrameRate = 30;
    // pVencChnAttr->stVencAttr.u32OutputFrameRate = 30;

    pVencChnAttr->stVencAttr.enMirrorFlip = DIRECTION_NONE;
    pVencChnAttr->stVencAttr.enRotation = CODEC_ROTATION_0;
    pVencChnAttr->stVencAttr.stCropCfg.bEnable = HB_FALSE;
    pVencChnAttr->stVencAttr.enPixelFormat = pixFmt;

    if (p_Width * p_Height > 2688 * 1522) {
        streambuf = 3 * 1024 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 7900*1024;
    } else if (p_Width * p_Height > 1920 * 1080) {
        streambuf = 2048 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 4*1024*1024;
    } else if (p_Width * p_Height > 1280 * 720) {
        streambuf = 1536 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 2100*1024;
    } else if (p_Width * p_Height > 704 * 576) {
        streambuf = 512 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 2100*1024;
    } else {
        streambuf = 256 * 1024;
        pVencChnAttr->stVencAttr.vlc_buf_size = 2048*1024;
    }
    // pVencChnAttr->stVencAttr.vlc_buf_size = 0;
    //     ((((p_Width * p_Height) * 9) >> 3) * 3) >> 1;
    if (p_enType == PT_JPEG || p_enType == PT_MJPEG) {
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 1;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = 2;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
        pVencChnAttr->stVencAttr.stAttrJpeg.dcf_enable = HB_FALSE;
        pVencChnAttr->stVencAttr.stAttrJpeg.quality_factor = 0;
        pVencChnAttr->stVencAttr.stAttrJpeg.restart_interval = 0;
        pVencChnAttr->stVencAttr.u32BitStreamBufSize = streambuf;
    } else {
        pVencChnAttr->stVencAttr.u32BitStreamBufferCount = 3;
        pVencChnAttr->stVencAttr.u32FrameBufferCount = 3;
        pVencChnAttr->stVencAttr.bExternalFreamBuffer = HB_TRUE;
        pVencChnAttr->stVencAttr.u32BitStreamBufSize = streambuf;
    }

    if (p_enType == PT_H265) {
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
        pVencChnAttr->stRcAttr.stH265Vbr.bQpMapEnable = HB_TRUE;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraQp = 20;
        pVencChnAttr->stRcAttr.stH265Vbr.u32IntraPeriod = 60;
        pVencChnAttr->stRcAttr.stH265Vbr.u32FrameRate = 30;
    }
    if (p_enType == PT_H264) {
        pVencChnAttr->stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
        pVencChnAttr->stRcAttr.stH264Vbr.bQpMapEnable = HB_TRUE;
        pVencChnAttr->stRcAttr.stH264Vbr.u32IntraQp = 20;
        pVencChnAttr->stRcAttr.stH264Vbr.u32IntraPeriod = 60;
        pVencChnAttr->stRcAttr.stH264Vbr.u32FrameRate = 30;
        pVencChnAttr->stVencAttr.stAttrH264.h264_profile = HB_H264_PROFILE_MP;
        pVencChnAttr->stVencAttr.stAttrH264.h264_level = HB_H264_LEVEL1;
    }
    pVencChnAttr->stGopAttr.u32GopPresetIdx = 2;
    pVencChnAttr->stGopAttr.s32DecodingRefreshType = 2;

    return 0;
}
int HobotVenc::Create(int width, int height, int bits)
{
    int s32Ret;
    VENC_RC_ATTR_S *pstRcParam;
    VENC_PARAM_MOD_S stModParam;
    m_nPicWidth = width;
    m_nPicHeight = height;

    VencChnAttrInit(&m_oVencChnAttr, m_enPalType, width, height, HB_PIXEL_FORMAT_NV12);

    s32Ret = HB_VENC_CreateChn(m_nCodecChn, &m_oVencChnAttr);
    if (s32Ret != 0) {
        ROS_printf(0, "HB_VENC_CreateChn %d failed, %d.\n", m_nCodecChn, s32Ret);
        return -1;
    }
    // stModParam.u32OneStreamBuffer = 0;
    // s32Ret = HB_VENC_SetModParam(m_nCodecChn, &stModParam);
    // if (s32Ret != 0) {
    //     ROS_printf("HB_VENC_SetModParam %d failed, %d.\n", m_nCodecChn, s32Ret);
    //     return -1;
    // }

    // HB_VENC_Module_Uninit();
    if (m_enPalType == PT_H264) {
        pstRcParam = &(m_oVencChnAttr.stRcAttr);
        s32Ret = HB_VENC_GetRcParam(m_nCodecChn, pstRcParam);
        if (s32Ret != 0) {
            ROS_printf(0, "HB_VENC_GetRcParam failed.\n");
            return -1;
        }
        switch (m_oVencChnAttr.stRcAttr.enRcMode) {
        case VENC_RC_MODE_H264CBR:
            venc_h264cbr(pstRcParam, bits, 30, 30, 3000);
            break;
        case VENC_RC_MODE_H264VBR:
            venc_h264vbr(pstRcParam, 30, 30, 30);
            break;
        case VENC_RC_MODE_H264AVBR:
            venc_h264avbr(pstRcParam, bits, 30, 30, 3000);
            break;
        case VENC_RC_MODE_H264FIXQP:
            venc_h264fixqp(pstRcParam, 30, 30, 0, 0, 0);
            break;
        case VENC_RC_MODE_H264QPMAP:
            venc_h264qpmap(pstRcParam, 30, 30);
            break;
        default:
            break;
        }
        ROS_printf(2, " m_oVencChnAttr.stRcAttr.enRcMode = %d\n",
                m_oVencChnAttr.stRcAttr.enRcMode);
    } else if (m_enPalType == PT_H265) {
        pstRcParam = &(m_oVencChnAttr.stRcAttr);
        m_oVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
        s32Ret = HB_VENC_GetRcParam(m_nCodecChn, pstRcParam);
        if (s32Ret != 0) {
            ROS_printf(0, "HB_VENC_GetRcParam failed.\n");
            return -1;
        }
        switch (m_oVencChnAttr.stRcAttr.enRcMode) {
        case VENC_RC_MODE_H265CBR:
            venc_h265cbr(pstRcParam, bits, 30, 30, 3000);
            break;
        case VENC_RC_MODE_H265VBR:
            venc_h265vbr(pstRcParam, 30, 30, 30);
            break;
        case VENC_RC_MODE_H265AVBR:
            venc_h265avbr(pstRcParam, bits, 30, 30, 3000);
            break;
        case VENC_RC_MODE_H265FIXQP:
            venc_h265fixqp(pstRcParam, 30, 30, 0, 0, 0);
            break;
        case VENC_RC_MODE_H265QPMAP:
            venc_h265qpmap(pstRcParam, 30, 30);
        default:
            break;
        }
        ROS_printf(2, " m_VencChnAttr.stRcAttr.enRcMode = %d\n",
                m_oVencChnAttr.stRcAttr.enRcMode);
    } else if (m_enPalType == PT_MJPEG) {
        pstRcParam = &(m_oVencChnAttr.stRcAttr);
        m_oVencChnAttr.stRcAttr.enRcMode = VENC_RC_MODE_MJPEGFIXQP;
        s32Ret = HB_VENC_GetRcParam(m_nCodecChn, pstRcParam);
        if (s32Ret != 0) {
            ROS_printf(0, "HB_VENC_GetRcParam failed.\n");
            return -1;
        }
        venc_mjpgfixqp(pstRcParam, 30, 50);
    }
    s32Ret = HB_VENC_SetChnAttr(m_nCodecChn, &m_oVencChnAttr);  // config
    if (s32Ret != 0) {
        ROS_printf(0, "HB_VENC_SetChnAttr failed\n");
        return -1;
    }
    ROS_printf(2, "[%s]->sucess chn=%d,ret=%d.\n", __func__, m_nCodecChn, s32Ret);

    return 0;
}
int HobotVenc::SetFps(int m_nCodecChn, int InputFps, int OutputFps)
{
    VENC_CHN_PARAM_S chnparam;

    HB_VENC_GetChnParam(m_nCodecChn, &chnparam);
    chnparam.stFrameRate.s32InputFrameRate = InputFps;
    chnparam.stFrameRate.s32OutputFrameRate = OutputFps;
    HB_VENC_SetChnParam(m_nCodecChn, &chnparam);
    return 0;
}

int HobotVenc::venc_h264cbr(VENC_RC_ATTR_S *pstRcParam, int bits,
        int framerate, int intraperiod, int vbvbuf)
{
    pstRcParam->stH264Cbr.u32BitRate = bits;
    pstRcParam->stH264Cbr.u32FrameRate = framerate;
    pstRcParam->stH264Cbr.u32IntraPeriod = intraperiod;
    pstRcParam->stH264Cbr.u32VbvBufferSize = vbvbuf;
    pstRcParam->stH264Cbr.u32IntraQp = 30;
    pstRcParam->stH264Cbr.u32InitialRcQp = 63;
    pstRcParam->stH264Cbr.bMbLevelRcEnable = HB_FALSE;
    pstRcParam->stH264Cbr.u32MaxIQp = 45;
    pstRcParam->stH264Cbr.u32MinIQp = 22;
    pstRcParam->stH264Cbr.u32MaxPQp = 45;
    pstRcParam->stH264Cbr.u32MinPQp = 22;
    pstRcParam->stH264Cbr.u32MaxBQp = 45;
    pstRcParam->stH264Cbr.u32MinBQp = 22;
    pstRcParam->stH264Cbr.bHvsQpEnable = HB_TRUE;
    pstRcParam->stH264Cbr.s32HvsQpScale = 2;
    pstRcParam->stH264Cbr.u32MaxDeltaQp = 10;
    pstRcParam->stH264Cbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int HobotVenc::venc_h264vbr(VENC_RC_ATTR_S *pstRcParam, int intraperiod,
            int intraqp, int framerate)
{
    pstRcParam->stH264Vbr.u32IntraPeriod = intraperiod;
    pstRcParam->stH264Vbr.u32IntraQp = intraqp;
    pstRcParam->stH264Vbr.u32FrameRate = framerate;
    pstRcParam->stH264Vbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int HobotVenc::venc_h264avbr(VENC_RC_ATTR_S *pstRcParam, int bits,
        int framerate, int intraperiod, int vbvbuf)
{
    pstRcParam->stH264AVbr.u32BitRate = bits;
    pstRcParam->stH264AVbr.u32FrameRate = framerate;
    pstRcParam->stH264AVbr.u32IntraPeriod = intraperiod;
    pstRcParam->stH264AVbr.u32VbvBufferSize = vbvbuf;
    pstRcParam->stH264AVbr.u32IntraQp = 30;
    pstRcParam->stH264AVbr.u32InitialRcQp = 63;
    pstRcParam->stH264AVbr.bMbLevelRcEnable = HB_FALSE;
    pstRcParam->stH264AVbr.u32MaxIQp = 45;
    pstRcParam->stH264AVbr.u32MinIQp = 22;
    pstRcParam->stH264AVbr.u32MaxPQp = 45;
    pstRcParam->stH264AVbr.u32MinPQp = 22;
    pstRcParam->stH264AVbr.u32MaxBQp = 45;
    pstRcParam->stH264AVbr.u32MinBQp = 22;
    pstRcParam->stH264AVbr.bHvsQpEnable = HB_TRUE;
    pstRcParam->stH264AVbr.s32HvsQpScale = 2;
    pstRcParam->stH264AVbr.u32MaxDeltaQp = 10;
    pstRcParam->stH264AVbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int HobotVenc::venc_h264fixqp(VENC_RC_ATTR_S *pstRcParam, int framerate,
            int intraperiod, int iqp, int pqp, int bqp)
{
    pstRcParam->stH264FixQp.u32FrameRate = framerate;
    pstRcParam->stH264FixQp.u32IntraPeriod = intraperiod;
    pstRcParam->stH264FixQp.u32IQp = iqp;
    pstRcParam->stH264FixQp.u32PQp = pqp;
    pstRcParam->stH264FixQp.u32BQp = bqp;

    return 0;
}

int HobotVenc::venc_h264qpmap(VENC_RC_ATTR_S *pstRcParam, int framerate,
                int intraperiod)
{
    pstRcParam->stH264QpMap.u32IntraPeriod = intraperiod;
    pstRcParam->stH264QpMap.u32FrameRate = framerate;
    pstRcParam->stH264QpMap.u32QpMapArrayCount =
        ((640+0x0f)&~0x0f)/16 * ((480+0x0f)&~0x0f)/16;
    pstRcParam->stH264QpMap.u32QpMapArray =
        (unsigned char*)malloc((pstRcParam->stH264QpMap. \
        u32QpMapArrayCount)*sizeof(unsigned char));
    memset(pstRcParam->stH264QpMap.u32QpMapArray, 30,
            pstRcParam->stH264QpMap.u32QpMapArrayCount);

    return 0;
}

int HobotVenc::venc_h265cbr(VENC_RC_ATTR_S *pstRcParam, int bits,
        int framerate, int intraperiod, int vbvbuf)
{
    pstRcParam->stH265Cbr.u32IntraPeriod = intraperiod;
    pstRcParam->stH265Cbr.u32IntraQp = 30;
    pstRcParam->stH265Cbr.u32BitRate = bits;
    pstRcParam->stH265Cbr.u32FrameRate = framerate;
    pstRcParam->stH265Cbr.u32InitialRcQp = 63;
    pstRcParam->stH265Cbr.u32VbvBufferSize = vbvbuf;
    pstRcParam->stH265Cbr.bCtuLevelRcEnable = HB_FALSE;
    pstRcParam->stH265Cbr.u32MaxIQp = 45;
    pstRcParam->stH265Cbr.u32MinIQp = 22;
    pstRcParam->stH265Cbr.u32MaxPQp = 45;
    pstRcParam->stH265Cbr.u32MinPQp = 22;
    pstRcParam->stH265Cbr.u32MaxBQp = 45;
    pstRcParam->stH265Cbr.u32MinBQp = 22;
    pstRcParam->stH265Cbr.bHvsQpEnable = HB_TRUE;
    pstRcParam->stH265Cbr.s32HvsQpScale = 2;
    pstRcParam->stH265Cbr.u32MaxDeltaQp = 10;
    pstRcParam->stH265Cbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int HobotVenc::venc_h265vbr(VENC_RC_ATTR_S *pstRcParam, int intraperiod,
            int intraqp, int framerate)
{
    pstRcParam->stH265Vbr.u32IntraPeriod = intraperiod;
    pstRcParam->stH265Vbr.u32IntraQp = intraqp;
    pstRcParam->stH265Vbr.u32FrameRate = framerate;
    pstRcParam->stH265Vbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int HobotVenc::venc_h265avbr(VENC_RC_ATTR_S *pstRcParam, int bits,
        int framerate, int intraperiod, int vbvbuf)
{
    pstRcParam->stH265Cbr.u32IntraPeriod = intraperiod;
    pstRcParam->stH265Cbr.u32IntraQp = 30;
    pstRcParam->stH265Cbr.u32BitRate = bits;
    pstRcParam->stH265Cbr.u32FrameRate = framerate;
    pstRcParam->stH265Cbr.u32InitialRcQp = 63;
    pstRcParam->stH265Cbr.u32VbvBufferSize = vbvbuf;
    pstRcParam->stH265Cbr.bCtuLevelRcEnable = HB_FALSE;
    pstRcParam->stH265Cbr.u32MaxIQp = 45;
    pstRcParam->stH265Cbr.u32MinIQp = 22;
    pstRcParam->stH265Cbr.u32MaxPQp = 45;
    pstRcParam->stH265Cbr.u32MinPQp = 22;
    pstRcParam->stH265Cbr.u32MaxBQp = 45;
    pstRcParam->stH265Cbr.u32MinBQp = 22;
    pstRcParam->stH265Cbr.bHvsQpEnable = HB_TRUE;
    pstRcParam->stH265Cbr.s32HvsQpScale = 2;
    pstRcParam->stH265Cbr.u32MaxDeltaQp = 10;
    pstRcParam->stH265Cbr.bQpMapEnable = HB_FALSE;

    return 0;
}

int HobotVenc::venc_h265fixqp(VENC_RC_ATTR_S *pstRcParam, int framerate,
            int intraperiod, int iqp, int pqp, int bqp)
{
    pstRcParam->stH265FixQp.u32FrameRate = framerate;
    pstRcParam->stH265FixQp.u32IntraPeriod = intraperiod;
    pstRcParam->stH265FixQp.u32IQp = iqp;
    pstRcParam->stH265FixQp.u32PQp = pqp;
    pstRcParam->stH265FixQp.u32BQp = bqp;

    return 0;
}

int HobotVenc::venc_h265qpmap(VENC_RC_ATTR_S *pstRcParam, int framerate,
                int intraperiod)
{
    pstRcParam->stH264QpMap.u32IntraPeriod = intraperiod;
    pstRcParam->stH264QpMap.u32FrameRate = framerate;
    pstRcParam->stH264QpMap.u32QpMapArrayCount =
        ((640+0x0f)&~0x0f)/16 * ((480+0x0f)&~0x0f)/16;
    pstRcParam->stH264QpMap.u32QpMapArray =
        (unsigned char*)malloc((pstRcParam->stH264QpMap. \
        u32QpMapArrayCount)*sizeof(unsigned char));
    memset(pstRcParam->stH264QpMap.u32QpMapArray, 30,
            pstRcParam->stH264QpMap.u32QpMapArrayCount);

    return 0;
}

int HobotVenc::venc_mjpgfixqp(VENC_RC_ATTR_S *pstRcParam, int framerate,
                int quality)
{
    pstRcParam->stMjpegFixQp.u32FrameRate = framerate;
    pstRcParam->stMjpegFixQp.u32QualityFactort = quality;
    ROS_printf(2, "[%s]->fps=%d, quality=%d.\n", __func__, framerate, quality);
    return 0;
}
/*
int read_nv12file(hb_vio_buffer_t *vio_buf, char* picFile, 
        int width, int height) {
    uint32_t size_y, size_uv;
    int img_in_fd;
    memset(vio_buf, 0, sizeof(hb_vio_buffer_t));
    size_y = width * height;
    size_uv = size_y / 2;
    prepare_user_buf_2lane(vio_buf, size_y, size_uv);
    vio_buf->img_info.planeCount = 2;
    vio_buf->img_info.img_format = 8;
    vio_buf->img_addr.width = width;
    vio_buf->img_addr.height = height;
    vio_buf->img_addr.stride_size = width;

    img_in_fd = open(picFile, O_RDWR | O_CREAT, 0644);
    if (img_in_fd < 0) {
        ROS_printf("open image:%s failed !\n", picFile);
        return -1;
    }

    read(img_in_fd, vio_buf->img_addr.addr[0], size_y);
    usleep(10 * 1000);
    read(img_in_fd, vio_buf->img_addr.addr[1], size_uv);
    usleep(10 * 1000);
    close(img_in_fd);
}*/

int HobotVenc::prepare_user_buf_2lane(void *buf, uint32_t size_y, uint32_t size_uv)
{
    /* ion_alloc_phy 找不到头文件，以及实现; grep -r -e "ion_alloc_phy" /usr/
    int ret;
    hb_vio_buffer_t *buffer = (hb_vio_buffer_t *)buf;
    if (buffer == NULL)
        return -1;
    buffer->img_info.fd[0] = ion_open();
    buffer->img_info.fd[1] = ion_open();
    ret  = ion_alloc_phy(size_y, &buffer->img_info.fd[0],
                        &buffer->img_addr.addr[0], &buffer->img_addr.paddr[0]);
    if (ret) {
        ROS_printf("prepare user buf error\n");
        return ret;
    }
    ret = ion_alloc_phy(size_uv, &buffer->img_info.fd[1],
                        &buffer->img_addr.addr[1], &buffer->img_addr.paddr[1]);
    if (ret) {
        ROS_printf("prepare user buf error\n");
        return ret;
    }
    ROS_printf("buf:y: vaddr = 0x%x paddr = 0x%x; uv: vaddr = 0x%x, paddr = 0x%x\n",
                buffer->img_addr.addr[0], buffer->img_addr.paddr[0],
                buffer->img_addr.addr[1], buffer->img_addr.paddr[1]);
    */
    return 0;
}
