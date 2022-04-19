// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/hobot_codec.h"
#include "include/hobot_vdec.h"
#include "include/hobot_venc.h"
#include <sys/times.h>

#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "include/video_utils.hpp"

// 返回ms
int32_t tool_calc_time_laps(const struct timespec &time_start, const struct timespec &time_end)
{
  int32_t nRetMs = 0;
  if (time_end.tv_nsec < time_start.tv_nsec)
  {
    nRetMs = (time_end.tv_sec - time_start.tv_sec - 1) * 1000 +
     (1000000000 + time_end.tv_nsec - time_start.tv_nsec) / 1000000;
  } else {
    nRetMs = (time_end.tv_sec - time_start.tv_sec) * 1000 + (time_end.tv_nsec - time_start.tv_nsec) / 1000000;
  }
  return nRetMs;
}

void TestSave(char *pFilePath, char *imgData, int nDlen)
{
  FILE *yuvFd = fopen(pFilePath, "w+");
  if (yuvFd) {
    fwrite(imgData, 1, nDlen, yuvFd);
    fclose(yuvFd);
  }
}
const char* enc_types[] = {
    "jpeg", "h264", "h265"};

const char* raw_types[] = {
    "bgr8", "rgb8", "nv12"};
// 解析命令，创建编解码器，发布接收节点
// ros2 run hobot_codec hobot_codec_republish ros jpeg decompress rgb8 --ros-args
// -p sub_topic:=/image_raw/compressed -p pub_topic:=/image_raw
// app trans_mode in_format work_mode in_topic:= out_topic:= out_format
/*
配置编解码类型及分辨率，目前不支持缩放，过来多大，就处理多大
多路同时工作，pipeid 需要处理 1-4 路
*/
int IsType(const char* tsType, const char **fmtTypes, int nArrLen)
{
  for (int nIdx = 0; nIdx < nArrLen; ++nIdx) {
    if (0 == strcmp(tsType, fmtTypes[nIdx]))
      return 1;
  }
  return 0;
}
void HobotCodec::get_params()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  for (auto & parameter : parameters_client->get_parameters(
      {"sub_topic", "pub_topic", "channel", "in_mode", "out_mode",
        "in_format", "out_format", "enc_qp", "jpg_quality"}))
  {
    if (parameter.get_name() == "sub_topic") {
      RCLCPP_INFO(rclcpp::get_logger("HobotCodec"),
        "sub_topic value: %s", parameter.value_to_string().c_str());
      in_sub_topic_ = parameter.value_to_string();
    } else if (parameter.get_name() == "pub_topic") {
      out_pub_topic_ = parameter.value_to_string();
    } else if (parameter.get_name() == "in_mode") {
      in_mode_ = parameter.value_to_string();
    } else if (parameter.get_name() == "out_mode") {
      out_mode_ = parameter.value_to_string();
    } else if (parameter.get_name() == "enc_qp") {
      RCLCPP_WARN(rclcpp::get_logger("HobotCodec"),
        "enc_qp: %f", parameter.as_double());
      enc_qp_ = parameter.as_double();
    } else if (parameter.get_name() == "jpg_quality") {
      RCLCPP_WARN(rclcpp::get_logger("HobotCodec"),
        "jpg_quality: %f", parameter.as_double());
      jpg_quality_ = parameter.as_double();
    } else if (parameter.get_name() == "in_format") {
      in_format_ = parameter.value_to_string();
      RCLCPP_INFO(rclcpp::get_logger("HobotCodec"),
        "image_height_ value: %s", parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "out_format") {
      out_format_ = parameter.value_to_string();
      RCLCPP_INFO(rclcpp::get_logger("HobotCodec"),
        "image_height_ value: %s", parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "channel") {
      mChannel_ = parameter.as_int();
      RCLCPP_INFO(rclcpp::get_logger("HobotCodec"),
        "mChannel_ value: %s", parameter.value_to_string().c_str());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("HobotCodec"),
        "Invalid parameter name: %s", parameter.get_name().c_str());
    }
  }
}
HobotCodec::HobotCodec(const rclcpp::NodeOptions& node_options,
  std::string node_name)
    : Node(node_name, node_options),
#ifdef H26X_CODEC
    img_pub_(new sensor_msgs::msg::Image()),
    frameh26x_sub_(new codec_msgs::msg::H26XFrame) {
#else
    img_pub_(new sensor_msgs::msg::Image()) {
#endif
  stop_ = false;
  this->declare_parameter("sub_topic", "/image_raw");
  this->declare_parameter("pub_topic", "/image_raw/compressed");
  this->declare_parameter("channel", 0);
  this->declare_parameter("in_mode", "ros");
  this->declare_parameter("out_mode", "ros");
  this->declare_parameter("in_format", "bgr8");
  this->declare_parameter("out_format", "jpeg");
  this->declare_parameter("enc_qp", 10.0);
  this->declare_parameter("jpg_quality", 60.0);
  get_params();

  RCLCPP_WARN(rclcpp::get_logger("HobotCodec"),
   "Args: %s %s %s %s", in_mode_.c_str(), out_mode_.c_str(), in_format_.c_str(), out_format_.c_str());
  init();
}

HobotCodec::~HobotCodec()
{
  stop_ = true;
  if (m_spThrdPub) {
    m_spThrdPub->join();
  }
  if (mPtrInNv12)
    delete[] mPtrInNv12;
  if (mPtrOutRGB2)
    delete[] mPtrOutRGB2;
  if (nullptr != m_pHwCodec) {
    m_pHwCodec->UninitCodec();
    delete m_pHwCodec;
  }
}
#define USE_THREAD
int HobotCodec::init()
{
  // 收到数据才可以知道 宽高，才能真正创建
  if (0 != out_format_.compare(in_format_)) {
    if (IsType(out_format_.c_str(), enc_types, 3)) {
      m_pHwCodec = new HobotVenc(mChannel_, out_format_.c_str());
    } else {
      m_pHwCodec = new HobotVdec(mChannel_, in_format_.c_str());
    }
    m_pHwCodec->InitCodec();
    m_pHwCodec->SetCodecAttr("jpg_quality", jpg_quality_);
    m_pHwCodec->SetCodecAttr("enc_qp", enc_qp_);
  }
  // 启动编解码器，成功则启动pub，帧率是一个问题，按最快速度执行
  // step1: create subscribe
  if (in_mode_.find("shared_mem") == std::string::npos) {
    RCLCPP_WARN(rclcpp::get_logger("HobotCodec"),
      "Create subscription with topic_name: %s", in_sub_topic_.c_str());
    if ((std::string::npos != in_format_.find("h264")) ||
      (std::string::npos != in_format_.find("h265"))) {
#ifdef H26X_CODEC
      ros_subh26xscription_ = this->create_subscription<codec_msgs::msg::H26XFrame>(
            in_sub_topic_, 10,
            std::bind(&HobotCodec::in_ros_h26x_topic_cb, this, std::placeholders::_1));
#endif
    } else {
      if (in_sub_topic_.compare(topic_name_compressed_) != 0) {
        ros_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            in_sub_topic_, 10,
            std::bind(&HobotCodec::in_ros_topic_cb, this, std::placeholders::_1));
      } else {
        ros_subscription_compressed_ =
            this->create_subscription<sensor_msgs::msg::CompressedImage>(
            in_sub_topic_, 10,
            std::bind(&HobotCodec::in_ros_compressed_cb, this,
                      std::placeholders::_1));
      }
    }
  } else {
#ifdef SHARED_MEM_MSG
    hbmem_subscription_ =
        this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            in_sub_topic_, 10,
            std::bind(&HobotCodec::in_hbmem_topic_cb, this,
                      std::placeholders::_1));
    RCLCPP_WARN(rclcpp::get_logger("HobotCodec"),
      "Create hbmem_subscription with topic_name: %s, sub = %p", in_sub_topic_.c_str(), hbmem_subscription_);
#endif
  }
  // step1: create publish
  const int period_ms = 1000.0 / framerate_;

#ifdef USE_THREAD
  m_spThrdPub = std::make_shared<std::thread>(std::bind(&HobotCodec::exec_loopPub, this));
#else
  if (out_mode_.compare("shared_mem") != 0) {
    ros_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(out_pub_topic_.c_str(), 10);

    RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "[ros]->pub 0x%x, topic=%s.\n",
      ros_image_publisher_, out_pub_topic_.c_str());
    timer_pub_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
      std::bind(&HobotCodec::timer_ros_pub, this));
  } else {
#ifdef SHARED_MEM_MSG
    publisher_hbmem_ = this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
        out_pub_topic_.c_str(), 10);
    timer_pub_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
      std::bind(&HobotCodec::timer_hbmem_pub, this));
#endif
  }
#endif
}
void HobotCodec::exec_loopPub() {
  const int period_ms = 1000.0 / framerate_;
  bool bHobot = false;
  if (out_mode_.compare("shared_mem") != 0) {
    ros_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(out_pub_topic_.c_str(), 10);
  } else {
    bHobot = true;
#ifdef SHARED_MEM_MSG
    publisher_hbmem_ = this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
        out_pub_topic_.c_str(), 10);
#endif
  }
  RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "[ros]->pub %s, topic=%s, period=%d.\n",
    out_format_.c_str(), out_pub_topic_.c_str(), period_ms);
  while (!stop_) {
    // 判断当前执行的模式
    if (bHobot) {
      timer_hbmem_pub();
    } else {
      timer_ros_pub();
    }
    usleep(period_ms*1000);
  }
}
// static int s_nSave = 0;
#ifdef SHARED_MEM_MSG
void HobotCodec::in_hbmem_topic_cb(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg) {
  struct timespec time_now = {0, 0}, time_end = {0, 0}, time_in = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  time_in.tv_nsec = msg->time_stamp.nanosec;
  time_in.tv_sec = msg->time_stamp.sec;
  uint64_t mNow = (time_now.tv_sec * 1000 + time_now.tv_nsec / 1000000);
  /* if (0 == s_nSave) {
    TestSave("/userdata/test.rgb", (char*)msg->data.data(), msg->data.size());
    s_nSave = 1;
  }
  */
  std::stringstream ss;
  ss << "in_hbmem_topic_cb img: " << msg->encoding.data()
  << ", stamp: " << msg->time_stamp.sec
  << "." << msg->time_stamp.nanosec;
  RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "%s", ss.str().c_str());
  if (nullptr != m_pHwCodec) {
    m_pHwCodec->Start(msg->width, msg->height);
    if (0 == in_format_.compare("bgr8") || 0 == in_format_.compare("rgb8")) {
      if (nullptr == mPtrInNv12)
        mPtrInNv12 = new uint8_t[msg->width * msg->height * 3 / 2];
      if (0 == in_format_.compare("bgr8") && mPtrInNv12) {
        video_utils::BGR24_to_NV12(msg->data.data(), mPtrInNv12, msg->width, msg->height);
      } else {
        video_utils::RGB24_to_NV12(msg->data.data(), mPtrInNv12, msg->width, msg->height);
      }
      m_pHwCodec->PutData(mPtrInNv12, msg->data_size, time_in);
    } else {
      m_pHwCodec->PutData(msg->data.data(), msg->data_size, time_in);
    }
    clock_gettime(CLOCK_REALTIME, &time_end);
    RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "[%s]->fmt=%s ,w:h=%d:%d,tmlaps %d ms,dLen=%d,laps %d.\n", __func__,
     msg->encoding.data(), msg->width, msg->height, tool_calc_time_laps(time_in, time_now), msg->data_size,
      (time_end.tv_sec * 1000 + time_end.tv_nsec / 1000000) - mNow);
  }
}
#endif

#ifdef H26X_CODEC
void HobotCodec::in_ros_h26x_topic_cb(
    const codec_msgs::msg::H26XFrame::ConstSharedPtr msg) {
  struct timespec time_now = {0, 0}, time_in = {0, 0}, time_end = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  time_in.tv_nsec = msg->dts.nanosec;
  time_in.tv_sec = msg->dts.sec;
  uint64_t mNow = (time_now.tv_sec * 1000 + time_now.tv_nsec / 1000000);
  {
    auto tp_raw_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_statraw_mtx_);
    sub_imgraw_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_raw_now - sub_imgraw_tp_).count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("HobotCodec"),
      "Sub imgRaw fps = %d", sub_imgraw_frameCount_);
      sub_imgraw_frameCount_ = 0;
      sub_imgraw_tp_ = std::chrono::system_clock::now();
    }
  }
  if (nullptr != m_pHwCodec) {
    m_pHwCodec->Start(msg->width, msg->height);
    m_pHwCodec->PutData(msg->data.data(), msg->data.size(), time_in);
    clock_gettime(CLOCK_REALTIME, &time_end);
    RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "[%s]->fmt=%s ,frm w:h=%d:%d, tmlaps %d ms,dLen=%d,laps %d.\n",
      __func__, msg->encoding.data(), msg->width, msg->height,
      tool_calc_time_laps(time_in, time_now), msg->data.size(),
      (time_end.tv_sec * 1000 + time_end.tv_nsec / 1000000) - mNow);
  }

  return;
}
#endif

void HobotCodec::in_ros_topic_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  struct timespec time_now = {0, 0}, time_in = {0, 0}, time_end = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  time_in.tv_nsec = msg->header.stamp.nanosec;
  time_in.tv_sec = msg->header.stamp.sec;
  uint64_t mNow = (time_now.tv_sec * 1000 + time_now.tv_nsec / 1000000);

  {
    auto tp_raw_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_statraw_mtx_);
    sub_imgraw_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_raw_now - sub_imgraw_tp_).count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("HobotCodec"),
      "Sub imgRaw fps = %d", sub_imgraw_frameCount_);
      sub_imgraw_frameCount_ = 0;
      sub_imgraw_tp_ = std::chrono::system_clock::now();
    }
  }
  if (nullptr != m_pHwCodec) {
    m_pHwCodec->Start(msg->width, msg->height);
    if (0 == in_format_.compare("bgr8") || 0 == in_format_.compare("rgb8")) {
      if (nullptr == mPtrInNv12)
        mPtrInNv12 = new uint8_t[msg->width * msg->height * 3 / 2];
      if (0 == in_format_.compare("bgr8") && mPtrInNv12) {
        video_utils::BGR24_to_NV12(msg->data.data(), mPtrInNv12, msg->width, msg->height);
      } else {
        video_utils::RGB24_to_NV12(msg->data.data(), mPtrInNv12, msg->width, msg->height);
      }
      m_pHwCodec->PutData(mPtrInNv12, msg->data.size(), time_in);
    } else {
      m_pHwCodec->PutData(msg->data.data(), msg->data.size(), time_in);
    }
    clock_gettime(CLOCK_REALTIME, &time_end);
    RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "[%s]->fmt=%s ,w:h=%d:%d,tmlaps %d ms,dLen=%d,laps %d.\n", __func__,
      msg->encoding.data(), msg->width, msg->height, tool_calc_time_laps(time_in, time_now), msg->data.size(),
      (time_end.tv_sec * 1000 + time_end.tv_nsec / 1000000) - mNow);
  }

  return;
}
void HobotCodec::in_ros_compressed_cb(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg) {
  struct timespec time_now = {0, 0}, time_in = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  time_in.tv_nsec = img_msg->header.stamp.nanosec;
  time_in.tv_sec = img_msg->header.stamp.sec;

  std::stringstream ss;
  ss << "Recv compressed img: " << img_msg->format
  // << ", w: " << img_msg->width
  // << ", h: " << img_msg->height
  << ", stamp: " << img_msg->header.stamp.sec
  << "." << img_msg->header.stamp.nanosec
  << ", tmlaps(ms): " << tool_calc_time_laps(time_in, time_now)
  << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "%s", ss.str().c_str());

  if (nullptr != m_pHwCodec) {
    m_pHwCodec->Start(1920, 1080);
    m_pHwCodec->PutData(img_msg->data.data(), img_msg->data.size(), time_in);
  }
}

// codec dec 和 enc 是不一样的接口？
void HobotCodec::timer_ros_pub()
{
  if (nullptr != m_pHwCodec) {
    TFrameData oFrame;
    if (0 == m_pHwCodec->GetFrame(&oFrame)) {
      struct timespec time_start = {0, 0}, time_end = {0, 0};
      clock_gettime(CLOCK_REALTIME, &time_start);
      if (0 == out_format_.compare("h264") ||
        0 == out_format_.compare("h265") ) {
#ifdef H26X_CODEC
        frameh26x_sub_->index = m_pHwCodec->GetFrameNum();
        frameh26x_sub_->dts.sec = oFrame.time_stamp.tv_sec;
        frameh26x_sub_->dts.nanosec = oFrame.time_stamp.tv_nsec;
        frameh26x_sub_->pts.sec = oFrame.time_stamp.tv_sec;
        frameh26x_sub_->pts.nanosec = oFrame.time_stamp.tv_nsec;
        memcpy(frameh26x_sub_->encoding.data(), out_format_.c_str(), out_format_.length());
        frameh26x_sub_->width = oFrame.mWidth;
        frameh26x_sub_->height = oFrame.mHeight;

        frameh26x_sub_->data.resize(oFrame.mDataLen);
        memcpy(&frameh26x_sub_->data[0], oFrame.mPtrData, oFrame.mDataLen);
        ros_image_publisher_->publish(*frameh26x_sub_);
#endif
      } else {
        img_pub_->header.stamp.sec = oFrame.time_stamp.tv_sec;
        img_pub_->header.stamp.nanosec = oFrame.time_stamp.tv_nsec;
        img_pub_->width = oFrame.mWidth;
        img_pub_->height = oFrame.mHeight;
        img_pub_->step = oFrame.mWidth;
        img_pub_->encoding = out_format_.c_str();
        if (HB_PIXEL_FORMAT_NV12 == oFrame.mFrameFmt) {
          if (0 == out_format_.compare("bgr8") || 0 == out_format_.compare("rgb8")) {
            int nRgbLen = oFrame.mHeight * oFrame.mWidth * 3;
            if (nullptr == mPtrInNv12)
              mPtrOutRGB2 = new uint8_t[nRgbLen];
            if (mPtrOutRGB2) {
              if (0 == out_format_.compare("bgr8")) {
                video_utils::NV12_TO_BGR24(oFrame.mPtrY, oFrame.mPtrUV, mPtrOutRGB2, oFrame.mWidth , oFrame.mHeight);
              } else {
                video_utils::NV12_TO_RGB24(oFrame.mPtrY, oFrame.mPtrUV, mPtrOutRGB2, oFrame.mWidth , oFrame.mHeight);
              }
              img_pub_->data.resize(nRgbLen);
              memcpy(&img_pub_->data[0], mPtrOutRGB2, nRgbLen);
            }
          } else {
            int nOffSet = oFrame.mHeight * oFrame.mWidth;
            img_pub_->data.resize(oFrame.mDataLen);
            memcpy(&img_pub_->data[0], oFrame.mPtrY, nOffSet);
            memcpy(&img_pub_->data[0] + nOffSet, oFrame.mPtrUV,
              nOffSet / 2);
          }
        } else {
          img_pub_->data.resize(oFrame.mDataLen);
          memcpy(&img_pub_->data[0], oFrame.mPtrData, oFrame.mDataLen);
        }
        std::stringstream ss;
        ss << "timer_ros_pub img: " << img_pub_->encoding
        // << ", w: " << img_msg->width
        // << ", h: " << img_msg->height
        << ", stamp: " << img_pub_->header.stamp.sec
        << "." << img_pub_->header.stamp.nanosec;
        RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "%s", ss.str().c_str());
        ros_image_publisher_->publish(*img_pub_);
      }
      m_pHwCodec->ReleaseFrame(&oFrame);

      clock_gettime(CLOCK_REALTIME, &time_end);
      RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "[%s_%d]->pub=0x%x,encoding=%s data %d, laps=%d.\n",
        __func__, __LINE__, ros_image_publisher_, out_format_.c_str(), oFrame.mDataLen,
        (time_end.tv_sec * 1000 + time_end.tv_nsec / 1000000) -
        (time_start.tv_sec * 1000 + time_start.tv_nsec / 1000000));
    }
  }
}
void HobotCodec::timer_hbmem_pub()
{
#ifdef SHARED_MEM_MSG
  if (nullptr != m_pHwCodec) {
    TFrameData oFrame;
    if (m_pHwCodec->GetFrame(&oFrame)) {
      auto loanedMsg = publisher_hbmem_->borrow_loaned_message();
      if (loanedMsg.is_valid()) {
        auto& msg = loanedMsg.get();
        // struct timespec time_start = {0, 0};
        // clock_gettime(CLOCK_REALTIME, &time_start);
        // msg.time_stamp = (time_start.tv_sec * 1000 + time_start.tv_nsec / 1000000);
        msg.time_stamp.sec = oFrame.time_stamp.tv_sec;
        msg.time_stamp.nanosec = oFrame.time_stamp.tv_nsec;
        memcpy(msg.encoding.data(), out_format_.c_str(), out_format_.length());
        msg.height = oFrame.mHeight;
        msg.width = oFrame.mWidth;
        msg.step = oFrame.mWidth;
        msg.data_size = oFrame.mDataLen;
        int nOffSet = oFrame.mHeight * oFrame.mWidth;
        if (HB_PIXEL_FORMAT_NV12 == oFrame.mFrameFmt) {
          if (0 == out_format_.compare("bgr8") || 0 == out_format_.compare("rgb8")) {
            int nRgbLen = oFrame.mHeight * oFrame.mWidth * 3;
            msg.data_size = nRgbLen;
            if (nullptr == mPtrInNv12)
              mPtrOutRGB2 = new uint8_t[nRgbLen];
            if (mPtrOutRGB2) {
              if (0 == out_format_.compare("bgr8")) {
                video_utils::NV12_TO_BGR24(oFrame.mPtrY, oFrame.mPtrUV, mPtrOutRGB2, oFrame.mWidth , oFrame.mHeight);
              } else {
                video_utils::NV12_TO_RGB24(oFrame.mPtrY, oFrame.mPtrUV, mPtrOutRGB2, oFrame.mWidth , oFrame.mHeight);
              }
              memcpy(msg.data.data(), mPtrOutRGB2, nRgbLen);
            }
          } else {
            memcpy(msg.data.data(), oFrame.mPtrY, nOffSet);
            memcpy(msg.data.data() + nOffSet, oFrame.mPtrUV,
              nOffSet / 2);
          }
        } else {
          memcpy(msg.data.data(), oFrame.mPtrData, oFrame.mDataLen);
        }
        msg.index = mSendIdx++;
        publisher_hbmem_->publish(std::move(loanedMsg));
        m_pHwCodec->ReleaseFrame(&oFrame);
        RCLCPP_INFO(rclcpp::get_logger("HobotCodec"), "[%s]->pub=0x%x,encoding=%s data %d.\n",
          __func__, publisher_hbmem_, msg.encoding.data(), oFrame.mDataLen);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("HobotCodec"), "borrow_loaned_message failed");
      }
    }
  }
#endif
}
