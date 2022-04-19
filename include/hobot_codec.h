// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#ifdef H26X_CODEC
#include "codec_msgs/msg/h26_x_frame.hpp"
#endif

#ifdef SHARED_MEM_MSG
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif
#include <thread>
#include <memory>

#include "include/hwcodec.h"

#ifndef INCLUDE_HOBOT_CODEC_H_
#define INCLUDE_HOBOT_CODEC_H_

using rclcpp::NodeOptions;
using ImgCbType =
  std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr &msg)>;

// 即是接收端，又是发布端，接收，传编解码器，得到结果pub，帧率？
class HobotCodec : public rclcpp::Node {
 public:
  explicit HobotCodec(const rclcpp::NodeOptions & node_options = NodeOptions(), std::string node_name = "hobot_codec");
  ~HobotCodec();

 private:
  int init();
  std::shared_ptr<std::thread> m_spThrdPub;
  std::atomic<bool> stop_;
  void exec_loopPub();
  /*
  接收，普通是 ROS image 格式；发布都是 image 格式，share mem 采用hbmmsg 格式
  */
  // ----------------------------------------------------------------- sub begin
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
    ros_subscription_ = nullptr;
  sensor_msgs::msg::Image::UniquePtr img_sub_;
#ifdef H26X_CODEC
  rclcpp::Subscription<codec_msgs::msg::H26XFrame>::ConstSharedPtr
    ros_subh26xscription_ = nullptr;
  codec_msgs::msg::H26XFrame::UniquePtr frameh26x_sub_;
#endif
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::ConstSharedPtr
    ros_subscription_compressed_ = nullptr;
  void in_ros_compressed_cb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);

#ifdef SHARED_MEM_MSG
  int32_t mSendIdx = 0;
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr
      hbmem_subscription_;
#endif
  void in_ros_topic_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
#ifdef H26X_CODEC
  void in_ros_h26x_topic_cb(const codec_msgs::msg::H26XFrame::ConstSharedPtr msg);
#endif

#ifdef SHARED_MEM_MSG
  void in_hbmem_topic_cb(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
#endif
  // ImgCbType img_cb_ = nullptr;
  // ----------------------------------------------------------------- sub end
  // ----------------------------------------------------------------- pub begin
  // shared image message
  sensor_msgs::msg::Image::UniquePtr img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_image_publisher_ = nullptr;
#ifdef SHARED_MEM_MSG
  rclcpp::TimerBase::SharedPtr timer_hbmem_;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr publisher_hbmem_;
#endif
  void timer_ros_pub();
  void timer_hbmem_pub();
  void get_params();
  rclcpp::TimerBase::SharedPtr timer_pub_;

  // ----------------------------------------------------------------- pub end
  // 订阅主题，进行转换并发布
  std::string topic_name_compressed_ = "/image_raw/compressed";
  std::string in_sub_topic_ = "/image_raw";
  std::string out_pub_topic_ = "/image_raw/compressed";
  std::string in_mode_ = "ros";    // 目前进是啥格式，出是啥格式，进 ros，出 ros，进 hbmem，出hbmem
  std::string out_mode_ = "hbmem";   //
  std::string in_format_ = "rgb8";
  std::string out_format_ = "jpeg";
  int framerate_ = 30;
  int mChannel_ = 0;
  float enc_qp_ = 10.0;
  float jpg_quality_ = 10.0;

  std::chrono::high_resolution_clock::time_point sub_imgraw_tp_;
  int sub_imgraw_frameCount_ = 0;
  std::mutex frame_statraw_mtx_;

 private:
  HWCodec *m_pHwCodec = nullptr;
  uint8_t *mPtrInNv12 = nullptr;
  uint8_t *mPtrOutRGB2 = nullptr;
};

#endif  // INCLUDE_HOBOT_CODEC_H_
