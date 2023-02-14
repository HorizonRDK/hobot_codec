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
#include <mutex>
#include <condition_variable>
#include <thread>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "img_msgs/msg/h26_x_frame.hpp"

#ifdef SHARED_MEM_MSG
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "hbm_img_msgs/msg/hbm_h26_x_frame.hpp"
#endif

#include "include/hobot_codec_impl.h"

#ifndef INCLUDE_HOBOT_CODEC_NODE_H_
#define INCLUDE_HOBOT_CODEC_NODE_H_

struct RunTimeData {
  // input stat
  float in_frame_count_ = 0;
  // units of delay are ms
  // communication delay
  float in_comm_delay_ = 0;
  // input codec delay
  float in_codec_delay_ = 0;

  // output stat
  float out_frame_count_ = 0;
  // units of delay are ms
  // delay from frame in to out
  float out_codec_delay_ = 0;
};

class RunTimeStat {
 public:
  static std::shared_ptr<RunTimeStat> GetInstance() {
    static auto sp_run_time_stat = std::make_shared<RunTimeStat>();
    return sp_run_time_stat;
  }

  int Update(std::shared_ptr<RunTimeData> sp_run_time_data);
  std::shared_ptr<RunTimeData> Get(int time_interval_ms = 1000);

 private:
  std::shared_ptr<std::chrono::high_resolution_clock::time_point>
      last_frame_tp_ = nullptr;
  RunTimeData run_time_data_;
  std::mutex frame_stat_mtx_;
};

// 即是接收端，又是发布端，接收，传编解码器，得到结果pub，帧率？
class HobotCodecNode : public rclcpp::Node {
 public:
  explicit HobotCodecNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions(),
  std::string node_name = "hobot_codec");
  ~HobotCodecNode();

 private:
  int init();
  std::shared_ptr<std::thread> m_spThrdPub = nullptr;
  void exec_loopPub(bool is_sharedmem_pub);
  /*
  接收，普通是 ROS image 格式；发布都是 image 格式，share mem 采用hbmmsg 格式
  */
  // ----------------------------------------------------------------- sub begin
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
    ros_subscription_ = nullptr;
  sensor_msgs::msg::Image::UniquePtr img_sub_;
  rclcpp::Subscription<img_msgs::msg::H26XFrame>::ConstSharedPtr
    ros_subscrip_h26x_ = nullptr;
  img_msgs::msg::H26XFrame::UniquePtr frameh26x_sub_ = nullptr;
  rclcpp::Publisher<img_msgs::msg::H26XFrame>::SharedPtr ros_h26ximage_publisher_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::ConstSharedPtr
    ros_subscription_compressed_ = nullptr;
  void in_ros_compressed_cb(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);

#ifdef SHARED_MEM_MSG
  int32_t mSendIdx = 0;
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr
      hbmem_subscription_ = nullptr;
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmH26XFrame>::SharedPtr
      hbmemH26x_subscription_;
#endif
  void in_ros_topic_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void in_ros_h26x_topic_cb(const img_msgs::msg::H26XFrame::ConstSharedPtr msg);

#ifdef SHARED_MEM_MSG
  void in_hbmem_topic_cb(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
  void in_hbmemh264_topic_cb(const hbm_img_msgs::msg::HbmH26XFrame::ConstSharedPtr msg);
#endif
  // shared image message
  sensor_msgs::msg::Image::UniquePtr img_pub_ = nullptr;
  sensor_msgs::msg::CompressedImage::UniquePtr compressed_img_pub_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_image_publisher_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr ros_compressed_image_publisher_ = nullptr;

#ifdef SHARED_MEM_MSG
  rclcpp::TimerBase::SharedPtr timer_hbmem_ = nullptr;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr hbmem_publisher_ = nullptr;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmH26XFrame>::SharedPtr h264hbmem_publisher_ = nullptr;
#endif
  void timer_ros_pub();
  void timer_hbmem_pub();
  //获取启动参数
  void get_params();
  //检查启动参数是否符合要求，不符合要求会输出error log，并退出进程
  void check_params();
  rclcpp::TimerBase::SharedPtr timer_pub_ = nullptr;

  // ----------------------------------------------------------------- pub end
  // 订阅主题，进行转换并发布
  std::string topic_name_compressed_ = "/image_raw/compressed";
  std::string in_sub_topic_ = "/image_raw";
  std::string out_pub_topic_ = "/image_raw/compressed";
  std::string in_mode_ = "ros";
  std::string out_mode_ = "shared_mem";
  std::string in_format_ = "rgb8";
  std::string out_format_ = "jpeg";
  int framerate_ = 30;
  int mChannel_ = 0;
  float enc_qp_ = 10.0;
  float jpg_quality_ = 10.0;
  bool dump_output_ = false;
  std::string dump_output_file_ = "dump_codec_output";

  std::chrono::high_resolution_clock::time_point sub_imgraw_tp_;
  int sub_imgraw_frameCount_ = 0;
  std::mutex frame_statraw_mtx_;

  uint64_t sub_frame_count_ = 0;
  uint64_t sub_frame_output_ = 0;
  int input_framerate_ = 30;
  int output_framerate_ = -1;

  uint64_t get_image_time = 0; //获取image时间
  std::mutex timestamp_mtx;
  rclcpp::TimerBase::SharedPtr get_timer; //每隔5s检查一次codec获取image的时间
  void on_get_timer();

 private:
  std::shared_ptr<HobotCodecImpl> sp_hobot_codec_impl_ = nullptr;
  std::shared_ptr<HobotCodecParaBase> sp_hobot_codec_data_info_ = nullptr;
  
  uint8_t *mPtrIn = nullptr;
  uint8_t *mPtrOut = nullptr;
};

#endif  // INCLUDE_HOBOT_CODEC_NODE_H_
