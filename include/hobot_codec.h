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
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#ifndef INCLUDE_HOBOT_CODEC_H_
#define INCLUDE_HOBOT_CODEC_H_

#include "img_msgs/msg/h26_x_frame.hpp"

#ifdef SHARED_MEM_MSG
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "hbm_img_msgs/msg/hbm_h26_x_frame.hpp"
#endif

#include <thread>
#include <memory>

#include "include/arrqueue.hpp"
#include "include/hwcodec.h"

typedef struct _tag_RecvImg {
  int mWidth;
  int mHeight;
  unsigned char* mPImgData;
  unsigned int mDataLen;
  char encoding[12];
  struct timespec time_stamp;
}TRecvImg;
#define FIFO_NUM 20
// #define THRD_CODEC_PUT

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
  void exec_loopPub();
#ifdef THRD_CODEC_PUT
  std::shared_ptr<std::thread> m_spThrdPut;
  void exec_loopPut();
  CArrayQueue<TRecvImg, FIFO_NUM> m_arrRecvImg;
  /*TRecvImg m_arrRecvImg[FIFO_NUM];
  unsigned int m_nTotalSubNum;
  unsigned int m_nCurPut;*/
#endif
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
  // ImgCbType img_cb_ = nullptr;
  // ----------------------------------------------------------------- sub end
  // ----------------------------------------------------------------- pub begin
  // shared image message
  sensor_msgs::msg::Image::UniquePtr img_pub_;
  sensor_msgs::msg::CompressedImage::UniquePtr compressed_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_image_publisher_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr ros_compressed_image_publisher_ = nullptr;

#ifdef SHARED_MEM_MSG
  rclcpp::TimerBase::SharedPtr timer_hbmem_;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr hbmem_publisher_;
  rclcpp::PublisherHbmem<hbm_img_msgs::msg::HbmH26XFrame>::SharedPtr h264hbmem_publisher_;
#endif
  void timer_ros_pub();
  void timer_hbmem_pub();
  void get_params();
  void check_params();
  rclcpp::TimerBase::SharedPtr timer_pub_;

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
  HWCodec *m_pHwCodec = nullptr;
  uint8_t *mPtrInNv12 = nullptr;
  uint8_t *mPtrOutRGB2 = nullptr;
#ifdef THRD_CODEC_PUT
  // tool
  void put_compressedimg_frame(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  void put_rosimg_frame(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void put_rosh26x_frame(const img_msgs::msg::H26XFrame::ConstSharedPtr msg);

#ifdef SHARED_MEM_MSG
  void put_hbm_frame(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
  void put_hbmh264_frame(const hbm_img_msgs::msg::HbmH26XFrame::ConstSharedPtr msg);
#endif
#endif
};

#endif  // INCLUDE_HOBOT_CODEC_H_
