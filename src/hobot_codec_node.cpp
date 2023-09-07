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

#include <sys/times.h>
#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "include/video_utils.hpp"
#include "hobot_codec_node.h"

#define PUB_QUEUE_NUM 5

const char* enc_types[] = {
    "jpeg", "jpeg-compressed", "h264", "h265"};

const char* raw_types[] = {
    "bgr8", "rgb8", "nv12"};

//接入数据传输的方式支持的类型
const char* in_mode[] = {
    "ros", "shared_mem"
};

//发出数据传输方式支持类型
const char* out_mode[] = {
    "ros", "shared_mem"
};

//支持订阅的数据格式
const char* in_format[] = {
  "bgr8", "rgb8", "nv12", "jpeg", "jpeg-compressed", "h264", "h265"
};

//支持的处理后发布数据格式
const char* out_format[] = {
  "bgr8","rgb8","nv12","jpeg","jpeg-compressed","h264","h265"
};

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
    RCLCPP_DEBUG(rclcpp::get_logger("HobotCodecNode"),
      "[IsType]->in %s - %d, %s - %d",
      tsType, strlen(tsType), fmtTypes[nIdx], strlen(fmtTypes[nIdx]));
    if (0 == strcmp(tsType, fmtTypes[nIdx]))
      return 1;
  }
  return 0;
}

void HobotCodecNode::get_params()
{
  RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"), "get_params");
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  for (auto &parameter : parameters_client->get_parameters(
           {"sub_topic", "pub_topic", "channel", "in_mode", "out_mode",
            "in_format", "out_format", "enc_qp", "jpg_quality",
            "input_framerate", "output_framerate", "dump_output"})) {
    if (parameter.get_name() == "sub_topic") {
      RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"),
        "sub_topic value: %s", parameter.value_to_string().c_str());
      in_sub_topic_ = parameter.value_to_string();
    } else if (parameter.get_name() == "pub_topic") {
      out_pub_topic_ = parameter.value_to_string();
    } else if (parameter.get_name() == "in_mode") {
      in_mode_ = parameter.value_to_string();
    } else if (parameter.get_name() == "out_mode") {
      out_mode_ = parameter.value_to_string();
    } else if (parameter.get_name() == "enc_qp") {
      RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"),
        "enc_qp: %f", parameter.as_double());
      enc_qp_ = parameter.as_double();
    } else if (parameter.get_name() == "jpg_quality") {
      RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"),
        "jpg_quality: %f", parameter.as_double());
      jpg_quality_ = parameter.as_double();
    } else if (parameter.get_name() == "in_format") {
      in_format_ = parameter.value_to_string();
      RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"),
        "in_format value: %s", parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "out_format") {
      out_format_ = parameter.value_to_string();
      RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"),
        "out_format value: %s", parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "channel") {
      mChannel_ = parameter.as_int();
      RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"),
        "mChannel_ value: %s", parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "input_framerate") {
      input_framerate_ = parameter.as_int();
      RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"),
        "input_framerate_ value: %s", parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "output_framerate") {
      output_framerate_ = parameter.as_int();
      RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"),
        "output_framerate_ value: %s", parameter.value_to_string().c_str());
    } else if (parameter.get_name() == "dump_output") {
      dump_output_ = parameter.as_bool();
      RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"),
        "dump_output_ value: %d, file: %s", dump_output_, dump_output_file_.data());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("HobotCodecNode"),
        "Invalid parameter name: %s", parameter.get_name().c_str());
    }
  }

  RCLCPP_WARN_STREAM(rclcpp::get_logger("HobotCodecNode"),
    "Parameters:"
    << "\nsub_topic: " << in_sub_topic_
    << "\npub_topic: " << out_pub_topic_
    << "\nchannel: " << mChannel_
    << "\nin_mode: " << in_mode_
    << "\nout_mode: " << out_mode_
    << "\nin_format: " << in_format_
    << "\nout_format: " << out_format_
    << "\nenc_qp: " << enc_qp_
    << "\njpg_quality: " << jpg_quality_
    << "\ninput_framerate: " << input_framerate_
    << "\noutput_framerate: " << output_framerate_
    << "\ndump_output: " << dump_output_
  );
}

void HobotCodecNode::check_params()
{
  if (mChannel_ < 0 || mChannel_ > 3) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"),
        "Invalid channel number: %d! 0~3 are supported, "
        "please check the channel parameter.", mChannel_);
    rclcpp::shutdown();
    return;
  }
  
  if (!IsType(in_mode_.c_str(), in_mode, 2)) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"),
    "Invalid in_mode: %s! 'ros' and 'shared_mem' are supported. "
    "Please check the in_mode parameter.", in_mode_.c_str());
    rclcpp::shutdown();
    return;
  }
  
  if (!IsType(out_mode_.c_str(), out_mode, 2)) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"),
    "Invalid out_mode: %s! 'ros' and 'shared_mem' are supported. "
    "Please check the out_mode parameter.", out_mode_.c_str());
    rclcpp::shutdown();
    return;
  }
  
  if (!IsType(in_format_.c_str(), in_format, 6)) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"),
    "Invalid in_format: %s! 'bgr8', 'rgb8', 'nv12', 'jpeg', 'jpeg-compressed', 'h264' "
    "and 'h265' are supported. Please check the in_format parameter.", in_format_.c_str());
    rclcpp::shutdown();
    return;
  }
  
  if (!IsType(out_format_.c_str(), out_format, 7)) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"),
    "Invalid out_format: %s! 'bgr8', 'rgb8', 'nv12', 'jpeg', 'jpeg-compressed', "
    "'h264' and 'h265' are supported. "
    "Please check the out_format parameter.", out_format_.c_str());
    rclcpp::shutdown();
    return;
  }

  if(IsType(in_format_.c_str(), enc_types, 4)) {
    if(!IsType(out_format_.c_str(), raw_types,3)) { // 输入为编码格式，输出必须为raw格式
      RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"),
        "%s cannot be decoded to %s", in_format_.c_str(), out_format_.c_str());
      rclcpp::shutdown();
      return;
    }
  } else if(IsType(in_format_.c_str(), raw_types,3)) {
    if(!IsType(out_format_.c_str(), enc_types, 4)) { // 输入为raw格式，输出必须为编码格式
      RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"),
        "%s cannot be encoded to %s", in_format_.c_str(), out_format_.c_str());
      rclcpp::shutdown();
      return;
    }
  }

  if (IsType(out_format_.c_str(), enc_types, 4)) {
    if (enc_qp_ < 0 || enc_qp_ > 100) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"),
      "Invalid enc_qp: %f! The value range is floating point number from 0 to 100."
      " Please check the enc_qp parameter.", enc_qp_);
      rclcpp::shutdown();
      return;
    }
    if (jpg_quality_ < 0 || jpg_quality_ > 100) {
      RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"),
      "Invalid jpg_quality: %f! The value range is floating point number from 0 to 100."
      " Please check the jpg_quality parameter.", jpg_quality_);
      rclcpp::shutdown();
      return;
    }
  }
  
  if (input_framerate_ <= 0) {
    RCLCPP_WARN(rclcpp::get_logger("HobotCodecNode"),
    "Invalid input_framerate: %d! The input_framerate must be a positive integer! "
    "Use '30' instead!", input_framerate_);
    input_framerate_ = 30;
  }
  if (output_framerate_ <=0 && output_framerate_ != -1) {
    RCLCPP_WARN(rclcpp::get_logger("HobotCodecNode"),
    "Invalid output_framerate: %d! The output_framerate must be a positive integer or '-1'! "
    "Use '-1' instead!", output_framerate_);
    output_framerate_ = -1;
  }
  if (output_framerate_ > input_framerate_) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"),
    "input_framerate: %d output_framerate: %d, "
    "output_framerate must be less than or equal to input_framerate",
    input_framerate_,
    output_framerate_);
    rclcpp::shutdown();
    return;
  }
}

void HobotCodecNode::on_get_timer()
{
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  uint64_t mNow = (time_now.tv_sec * 1000 + time_now.tv_nsec / 1000000);
  std::unique_lock<std::mutex> timestamp_lk(timestamp_mtx);
  //定时器每隔5s检查一次获取图片的时间，如果已经超过5s还未收到，会输出error日志。
  if (mNow - get_image_time >= 5000) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"),
                 "Hobot_Codec has not received image for more than 5 seconds!"
                 " Please check whether the image publisher still exists by "
                 "'ros2 topic info %s'!",
                 in_sub_topic_.c_str());
  }
  timestamp_lk.unlock();
}

HobotCodecNode::HobotCodecNode(const rclcpp::NodeOptions& node_options,
  std::string node_name)
    : Node(node_name, node_options),
    img_pub_(new sensor_msgs::msg::Image()),
    compressed_img_pub_(new sensor_msgs::msg::CompressedImage()),
    frameh26x_sub_(new img_msgs::msg::H26XFrame) {
  this->declare_parameter("sub_topic", "/image_raw");
  this->declare_parameter("pub_topic", "/image_raw/compressed");
  this->declare_parameter("channel", 0);
  this->declare_parameter("in_mode", "ros");
  this->declare_parameter("out_mode", "ros");
  this->declare_parameter("in_format", "bgr8");
  this->declare_parameter("out_format", "jpeg");
  this->declare_parameter("enc_qp", 10.0);
  this->declare_parameter("jpg_quality", 60.0);
  this->declare_parameter("input_framerate", 30);
  this->declare_parameter("output_framerate", -1);
  this->declare_parameter("dump_output", false);

  // 更新配置参数
  get_params();

  // 校验参数有效性，如果校验失败直接退出程序
  check_params();

  // 初始化：创建codec & 创建消息的发布者和订阅者
  if (0 != init()) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"),
      "init fail!");
    rclcpp::shutdown();
    return;
  }
}

HobotCodecNode::~HobotCodecNode()
{
  if (sp_hobot_codec_impl_) {
    sp_hobot_codec_impl_->DeInit();
  }

  if (m_spThrdPub) {
    m_spThrdPub->join();
  }

  if (mPtrIn) {
    delete[] mPtrIn;
    mPtrIn = nullptr;
  }
  if (mPtrOut)
    delete[] mPtrOut;
}

int HobotCodecNode::init()
{
  // check_params()中已经校验过输入输出图片格式，确保能够压缩或者解压缩成功

  // 创建codec数据实例
  sp_hobot_codec_data_info_ = std::make_shared<HobotCodecParaBase>();
  sp_hobot_codec_data_info_->in_format_ = in_format_;
  sp_hobot_codec_data_info_->out_format_ = out_format_;
  sp_hobot_codec_data_info_->mChannel_ = mChannel_;

  // 收到数据才可以知道 宽高，才能真正创建
  // 此处只创建实例，订阅到消息后根据具体分辨率进行codec的初始化
  if (IsType(out_format_.c_str(), enc_types, 4)) {
    // 输出是压缩格式，需要创建编码器
    sp_hobot_codec_data_info_->hobot_codec_type = HobotCodecType::ENCODER;
    sp_hobot_codec_data_info_->jpg_quality_ = jpg_quality_;
    sp_hobot_codec_data_info_->enc_qp_ = enc_qp_;
  } else {
    // 输出是非压缩格式，需要创建解码器
    sp_hobot_codec_data_info_->hobot_codec_type = HobotCodecType::DECODER;
  }

  RCLCPP_INFO(rclcpp::get_logger("HobotVdecNode"),
  "in_format: %s, out_format: %s, hobot_codec_type(1 encoder, 2 decoder): %d",
  in_format_.c_str(), out_format_.c_str(),
  static_cast<int>(sp_hobot_codec_data_info_->hobot_codec_type));

  // 创建codec实例并初始化
  sp_hobot_codec_impl_ = std::make_shared<HobotCodecImpl>();
  int ret = sp_hobot_codec_impl_->Init(sp_hobot_codec_data_info_);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"),
      "Hobot codec init fail! ret: %d", ret);
    return ret;
  }

  //定时检查codec输入状态，如果超时没有输入，报error日志
  get_timer = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int64_t>(5000)),
      std::bind(&HobotCodecNode::on_get_timer, this));

  // 创建消息的订阅者
  if (in_mode_.compare("shared_mem") != 0) {
    if ((std::string::npos != in_format_.find("h264")) ||
      (std::string::npos != in_format_.find("h265"))) {
      ros_subscrip_h26x_ = this->create_subscription<img_msgs::msg::H26XFrame>(
            in_sub_topic_, PUB_QUEUE_NUM,
            std::bind(&HobotCodecNode::in_ros_h26x_topic_cb, this, std::placeholders::_1));
    } else {
      if (in_format_.compare("jpeg-compressed") != 0) {
        // in_format_为bgr8/rgb8/nv12/jpeg，订阅消息类型为sensor_msgs::msg::Image
        ros_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            in_sub_topic_, PUB_QUEUE_NUM,
            std::bind(&HobotCodecNode::in_ros_topic_cb, this, std::placeholders::_1));
      } else {
        // in_format_为jpeg-compressed，订阅消息类型为sensor_msgs::msg::CompressedImage
        ros_subscription_compressed_ =
            this->create_subscription<sensor_msgs::msg::CompressedImage>(
            in_sub_topic_, PUB_QUEUE_NUM,
            std::bind(&HobotCodecNode::in_ros_compressed_cb, this,
                      std::placeholders::_1));
      }
    }
  } else {
#ifdef SHARED_MEM_MSG
    if ((std::string::npos != in_format_.find("h264")) ||
      (std::string::npos != in_format_.find("h265"))) {
      hbmemH26x_subscription_ = this->create_subscription_hbmem<hbm_img_msgs::msg::HbmH26XFrame>(
        in_sub_topic_, PUB_QUEUE_NUM,
        std::bind(&HobotCodecNode::in_hbmemh264_topic_cb, this, std::placeholders::_1));
    } else {
      hbmem_subscription_ = this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
        in_sub_topic_, PUB_QUEUE_NUM,
        std::bind(&HobotCodecNode::in_hbmem_topic_cb, this, std::placeholders::_1));
    }
#endif
  }
  
  // 创建消息的发布者
  bool is_sharedmem_pub = false;
  if (out_mode_.compare("shared_mem") != 0) {
    if (0 == out_format_.compare("h264") ||
      0 == out_format_.compare("h265") ) {
      ros_h26ximage_publisher_ = this->create_publisher<img_msgs::msg::H26XFrame>(
        out_pub_topic_.c_str(), PUB_QUEUE_NUM);
    } else if (0 == out_format_.compare("jpeg-compressed")) {
      ros_compressed_image_publisher_ =
          this->create_publisher<sensor_msgs::msg::CompressedImage>(
              out_pub_topic_, PUB_QUEUE_NUM);
    } else {
      ros_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(out_pub_topic_.c_str(), PUB_QUEUE_NUM);
    }
  } else {
    is_sharedmem_pub = true;
#ifdef SHARED_MEM_MSG
    if (0 == out_format_.compare("h264") ||
      0 == out_format_.compare("h265") ) {
      h264hbmem_publisher_ = this->create_publisher_hbmem<hbm_img_msgs::msg::HbmH26XFrame>(
        out_pub_topic_.c_str(), PUB_QUEUE_NUM);
    } else {
      hbmem_publisher_ = this->create_publisher_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
        out_pub_topic_.c_str(), PUB_QUEUE_NUM);
    }
#endif
  }

  // 创建发布线程
  m_spThrdPub = std::make_shared<std::thread>(
    std::bind(&HobotCodecNode::exec_loopPub, this, is_sharedmem_pub));
  return 0;
}

void HobotCodecNode::exec_loopPub(bool is_sharedmem_pub) {
  while (rclcpp::ok()) {
    // 判断当前执行的模式
    if (is_sharedmem_pub) {
      timer_hbmem_pub();
    } else {
      timer_ros_pub();
    }
  }
}

#ifdef SHARED_MEM_MSG
void HobotCodecNode::in_hbmemh264_topic_cb(
    const hbm_img_msgs::msg::HbmH26XFrame::ConstSharedPtr msg) {
  if (!rclcpp::ok()) {
    return;
  }

  if (!sp_hobot_codec_impl_) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"), "Invalid hobot codec impl");
    return;
  }

  struct timespec time_now = {0, 0}, time_end = {0, 0}, time_in = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  time_in.tv_nsec = msg->dts.nanosec;
  time_in.tv_sec = msg->dts.sec;
  uint64_t mNow = (time_now.tv_sec * 1000 + time_now.tv_nsec / 1000000);
  std::unique_lock<std::mutex> timestamp_lk(timestamp_mtx);
  get_image_time = mNow;
  timestamp_lk.unlock();

  if (0 != in_format_.compare(reinterpret_cast<const char*>(msg->encoding.data()))) {
    RCLCPP_WARN(rclcpp::get_logger("HobotCodecNode"), "[%s]->infmt err %s-%s",
      __func__, in_format_.c_str(), msg->encoding.data());
    return;
  }
  
  std::stringstream ss;
  ss << "recved img"
  << ", index: " << msg->index
  << ", encoding: " << msg->encoding.data()
  << ", w: " << msg->width
  << ", h: " << msg->height
  << ", size: " << msg->data_size
  << ", stamp: " << msg->dts.sec
  << "." << msg->dts.nanosec;
  
  if (0 == sp_hobot_codec_impl_->Input(msg->data.data(), msg->width, msg->height, msg->data_size,
    std::make_shared<FrameInfo>(msg->index, time_in, time_now))) {
    clock_gettime(CLOCK_REALTIME, &time_end);
    auto sp_run_time_data = std::make_shared<RunTimeData>();
    sp_run_time_data->in_frame_count_ = 1;
    sp_run_time_data->in_comm_delay_ =
      tool_calc_time_laps(time_in, time_now);
    sp_run_time_data->in_codec_delay_ =
      (time_end.tv_sec * 1000 + time_end.tv_nsec / 1000000) - mNow;
    RunTimeStat::GetInstance()->Update(sp_run_time_data);

    ss << ", comm delay ms: " << sp_run_time_data->in_comm_delay_
      << ", Input delay ms: " << sp_run_time_data->in_codec_delay_;
    RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"), "%s", ss.str().data());
  }

}

void HobotCodecNode::in_hbmem_topic_cb(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg) {
  if (!rclcpp::ok()) {
    return;
  }

  struct timespec time_now = {0, 0}, time_end = {0, 0}, time_in = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  time_in.tv_nsec = msg->time_stamp.nanosec;
  time_in.tv_sec = msg->time_stamp.sec;
  uint64_t mNow = (time_now.tv_sec * 1000 + time_now.tv_nsec / 1000000);
  std::unique_lock<std::mutex> timestamp_lk(timestamp_mtx);
  get_image_time = mNow;
  timestamp_lk.unlock();

  std::stringstream ss;
  ss << "recved img"
  << ", index: " << msg->index
  << ", encoding: " << msg->encoding.data()
  << ", w: " << msg->width
  << ", h: " << msg->height
  << ", size: " << msg->data_size
  << ", stamp: " << msg->time_stamp.sec
  << "." << msg->time_stamp.nanosec;

  if (0 != in_format_.compare(reinterpret_cast<const char*>(msg->encoding.data()))) {
    RCLCPP_WARN(rclcpp::get_logger("HobotCodecNode"), "Recved img encoding: %s is unmatch with setting: %s",
      msg->encoding.data(), in_format_.c_str());
    return;
  }

  sub_frame_count_++;
  if (output_framerate_ > 0) {
    sub_frame_output_ += output_framerate_;
    if (static_cast<int>(sub_frame_output_) >= input_framerate_) {
      sub_frame_output_ -= input_framerate_;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"),
                  "[%s]->drop %d, input %d, output %d, %d", __func__,
                  sub_frame_count_, input_framerate_, output_framerate_,
                  sub_frame_output_);
      return;
    }
  }

  if (!sp_hobot_codec_impl_) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("HobotCodecNode"), "Invalid codec impl!");
    return;
  }
#ifndef PLATFORM_X86
  if (0 == in_format_.compare("bgr8") || 0 == in_format_.compare("rgb8")) {
    int nYuvLen = msg->width * msg->height * 3 / 2;
    if (nullptr == mPtrIn)
      mPtrIn = new uint8_t[msg->width * msg->height * 3 / 2];
    if (0 == in_format_.compare("bgr8") && mPtrIn) {
      video_utils::BGR24_to_NV12(msg->data.data(), mPtrIn, msg->width, msg->height);
    } else {
      video_utils::RGB24_to_NV12(msg->data.data(), mPtrIn, msg->width, msg->height);
    }
    sp_hobot_codec_impl_->Input(mPtrIn, msg->width, msg->height, nYuvLen,
      std::make_shared<FrameInfo>(msg->index, time_in, time_now));
  } else {
    sp_hobot_codec_impl_->Input(msg->data.data(), msg->width, msg->height, msg->data_size,
      std::make_shared<FrameInfo>(msg->index, time_in, time_now));
  }
#else
  if (nullptr == mPtrIn) {
    mPtrIn = new uint8_t[msg->width * msg->height * 3];
  } 
  //opencv编码输入类型为BGR格式
  if (0 == in_format_.compare("rgb8") && mPtrIn) {
    video_utils::RGB24_to_BGR24(msg->data.data(), mPtrIn, msg->width, msg->height);
    sp_hobot_codec_impl_->Input(mPtrIn, msg->width, msg->height, msg->width * msg->height * 3, std::make_shared<FrameInfo>(0, time_in, time_now));
  } else if (0 == in_format_.compare("nv12")) {
    video_utils::NV12_to_BGR24(msg->data.data(), mPtrIn, msg->width, msg->height);
    sp_hobot_codec_impl_->Input(mPtrIn, msg->width, msg->height, msg->width * msg->height * 3 / 2, std::make_shared<FrameInfo>(0, time_in, time_now));
  } else {
    //jpeg的解码以及BGR8的编码调用该接口
    sp_hobot_codec_impl_->Input(msg->data.data(), msg->width, msg->height, msg->data_size, std::make_shared<FrameInfo>(0, time_in, time_now));    
  }
#endif

  clock_gettime(CLOCK_REALTIME, &time_end);
  
  auto sp_run_time_data = std::make_shared<RunTimeData>();
  sp_run_time_data->in_frame_count_ = 1;
  sp_run_time_data->in_comm_delay_ =
    tool_calc_time_laps(time_in, time_now);
  sp_run_time_data->in_codec_delay_ =
    (time_end.tv_sec * 1000 + time_end.tv_nsec / 1000000) - mNow;
  RunTimeStat::GetInstance()->Update(sp_run_time_data);

  ss << ", comm delay ms: " << sp_run_time_data->in_comm_delay_
    << ", Input delay ms: " << sp_run_time_data->in_codec_delay_;
  RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"), "%s", ss.str().data());
}
#endif

void HobotCodecNode::in_ros_h26x_topic_cb(
    const img_msgs::msg::H26XFrame::ConstSharedPtr msg) {
  if (!rclcpp::ok()) {
    return;
  }

  if (!sp_hobot_codec_impl_) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"), "Invalid hobot codec impl");
    return;
  }

  struct timespec time_now = {0, 0}, time_in = {0, 0}, time_end = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  time_in.tv_nsec = msg->dts.nanosec;
  time_in.tv_sec = msg->dts.sec;
  uint64_t mNow = (time_now.tv_sec * 1000 + time_now.tv_nsec / 1000000);
  std::unique_lock<std::mutex> timestamp_lk(timestamp_mtx);
  get_image_time = mNow;
  timestamp_lk.unlock();

  std::stringstream ss;
  ss << "recved img"
  << ", index: " << msg->index
  << ", encoding: " << msg->encoding.data()
  << ", w: " << msg->width
  << ", h: " << msg->height
  << ", size: " << msg->data.size()
  << ", stamp: " << msg->dts.sec
  << "." << msg->dts.nanosec;

  if (0 != in_format_.compare(reinterpret_cast<const char*>(msg->encoding.data()))) {
    RCLCPP_WARN(rclcpp::get_logger("HobotCodecNode"), "Recved img encoding: %s is unmatch with setting: %s",
      msg->encoding.data(), in_format_.c_str());
    return;
  }
  {
    auto tp_raw_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_statraw_mtx_);
    sub_imgraw_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_raw_now - sub_imgraw_tp_).count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("HobotCodecNode"),
      "Sub imgRaw fps = %d", sub_imgraw_frameCount_);
      sub_imgraw_frameCount_ = 0;
      sub_imgraw_tp_ = std::chrono::system_clock::now();
    }
  }
  if (0 == sp_hobot_codec_impl_->Input(msg->data.data(), msg->width, msg->height, msg->data.size(),
    std::make_shared<FrameInfo>(msg->index, time_in, time_now))) {
    clock_gettime(CLOCK_REALTIME, &time_end);
    auto sp_run_time_data = std::make_shared<RunTimeData>();
    sp_run_time_data->in_frame_count_ = 1;
    sp_run_time_data->in_comm_delay_ =
      tool_calc_time_laps(time_in, time_now);
    sp_run_time_data->in_codec_delay_ =
      (time_end.tv_sec * 1000 + time_end.tv_nsec / 1000000) - mNow;
    RunTimeStat::GetInstance()->Update(sp_run_time_data);

    ss << ", comm delay ms: " << sp_run_time_data->in_comm_delay_
      << ", Input delay ms: " << sp_run_time_data->in_codec_delay_;
    RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"), "%s", ss.str().data());
  }

  return;
}

void HobotCodecNode::in_ros_topic_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  if (!rclcpp::ok()) {
    return;
  }
  
  if (!sp_hobot_codec_impl_) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"), "Invalid hobot codec impl");
    return;
  }

  struct timespec time_now = {0, 0}, time_in = {0, 0}, time_end = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  time_in.tv_nsec = msg->header.stamp.nanosec;
  time_in.tv_sec = msg->header.stamp.sec;
  uint64_t mNow = (time_now.tv_sec * 1000 + time_now.tv_nsec / 1000000);
  std::unique_lock<std::mutex> timestamp_lk(timestamp_mtx);
  get_image_time = mNow;
  timestamp_lk.unlock();

  if (0 != in_format_.compare(reinterpret_cast<const char*>(msg->encoding.data()))) {
    RCLCPP_WARN(rclcpp::get_logger("HobotCodecNode"), "[%s]->infmt err %s-%s",
      __func__, in_format_.c_str(), msg->encoding.data());
    return;
  }
  {
    auto tp_raw_now = std::chrono::system_clock::now();
    std::unique_lock<std::mutex> lk(frame_statraw_mtx_);
    sub_imgraw_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_raw_now - sub_imgraw_tp_).count();
    if (interval >= 1000) {
      RCLCPP_WARN(rclcpp::get_logger("HobotCodecNode"),
      "Sub imgRaw fps = %d", sub_imgraw_frameCount_);
      sub_imgraw_frameCount_ = 0;
      sub_imgraw_tp_ = std::chrono::system_clock::now();
    }
  }

  sub_frame_count_++;
  if (output_framerate_ > 0) {
    sub_frame_output_ += output_framerate_;
    if (static_cast<int>(sub_frame_output_) >= input_framerate_) {
      sub_frame_output_ -= input_framerate_;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"),
                  "[%s]->drop %d, input %d, output %d, %d", __func__,
                  sub_frame_count_, input_framerate_, output_framerate_,
                  sub_frame_output_);
      return;
    }
  }
#ifndef PLATFORM_X86

  if (0 == in_format_.compare("bgr8") || 0 == in_format_.compare("rgb8")) {
    size_t nYuvLen = msg->width * msg->height * 3 / 2;
    if (msg->data.size() != nYuvLen * 2) {
      RCLCPP_WARN(rclcpp::get_logger("HobotCodecNode"), "[%s]->inlen err %d-%d",
        __func__, msg->data.size(), nYuvLen * 2);
      return;
    }
    if (nullptr == mPtrIn)
      mPtrIn = new uint8_t[nYuvLen];
    if (0 == in_format_.compare("bgr8") && mPtrIn) {
      video_utils::BGR24_to_NV12(msg->data.data(), mPtrIn, msg->width, msg->height);
    } else {
      video_utils::RGB24_to_NV12(msg->data.data(), mPtrIn, msg->width, msg->height);
    }
    sp_hobot_codec_impl_->Input(mPtrIn, msg->width, msg->height, nYuvLen,
      std::make_shared<FrameInfo>(0, time_in, time_now));
  } else {
    sp_hobot_codec_impl_->Input(msg->data.data(), msg->width, msg->height, msg->data.size(),
      std::make_shared<FrameInfo>(0, time_in, time_now));
  }

#else

  if (nullptr == mPtrIn) {
    mPtrIn = new uint8_t[msg->width * msg->height * 3];
  } 
  //opencv编码输入类型为BGR格式
  if (0 == in_format_.compare("rgb8") && mPtrIn) {
    video_utils::RGB24_to_BGR24(msg->data.data(), mPtrIn, msg->width, msg->height);
    sp_hobot_codec_impl_->Input(mPtrIn, msg->width, msg->height, msg->width * msg->height * 3, std::make_shared<FrameInfo>(0, time_in, time_now));
  } else if (0 == in_format_.compare("nv12")) {
    video_utils::NV12_to_BGR24(msg->data.data(), mPtrIn, msg->width, msg->height);
    sp_hobot_codec_impl_->Input(mPtrIn, msg->width, msg->height, msg->width * msg->height * 3 / 2, std::make_shared<FrameInfo>(0, time_in, time_now));
  } else {
    //jpeg的解码以及BGR8的编码调用该接口
    sp_hobot_codec_impl_->Input(msg->data.data(), msg->width, msg->height, msg->data.size(), std::make_shared<FrameInfo>(0, time_in, time_now));    
  }

#endif

  clock_gettime(CLOCK_REALTIME, &time_end);
  RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"), "recved img fmt: %s, w:h: %d:%d, tmlaps: %dms, dLen: %d, laps: %d.",
    msg->encoding.data(), msg->width, msg->height, tool_calc_time_laps(time_in, time_now),
    msg->data.size(), (time_end.tv_sec * 1000 + time_end.tv_nsec / 1000000) - mNow);
  return;
}

void HobotCodecNode::in_ros_compressed_cb(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_msg) {
  if (!rclcpp::ok()) {
    return;
  }

  if (!sp_hobot_codec_impl_) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"), "Invalid hobot codec impl");
    return;
  }

  struct timespec time_now = {0, 0}, time_in = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  time_in.tv_nsec = img_msg->header.stamp.nanosec;
  time_in.tv_sec = img_msg->header.stamp.sec;

  uint64_t mNow = (time_now.tv_sec * 1000 + time_now.tv_nsec / 1000000);
  std::unique_lock<std::mutex> timestamp_lk(timestamp_mtx);
  get_image_time = mNow;
  timestamp_lk.unlock();

  RCLCPP_INFO_STREAM(rclcpp::get_logger("HobotCodecNode"),
    "Recv compressed img: " << img_msg->format
    // << ", w: " << img_msg->width
    // << ", h: " << img_msg->height
    << ", stamp: " << img_msg->header.stamp.sec
    << "." << img_msg->header.stamp.nanosec
    << ", tmlaps(ms): " << tool_calc_time_laps(time_in, time_now)
    << ", size: " << img_msg->data.size()
  );

  // TODO 20230117 使用实际的分辨率
  sp_hobot_codec_impl_->Input(img_msg->data.data(), 1920, 1080, img_msg->data.size(),
    std::make_shared<FrameInfo>(0, time_in, time_now));
}
// static struct timespec time_last;
// codec dec 和 enc 是不一样的接口？
void HobotCodecNode::timer_ros_pub()
{
  if (!rclcpp::ok()) {
    return;
  }

  if (!sp_hobot_codec_impl_) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"), "Invalid hobot codec impl");
    return;
  }

  auto oFrame = sp_hobot_codec_impl_->GetOutput();
  if (!oFrame) {
    if (rclcpp::ok()) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("HobotCodecNode"), "GetOutput fail!");
    }
    return;
  }

  std::stringstream ss;
  ss << "pub img"
    << ", index: " << oFrame->sp_frame_info->img_idx_;

  if (0 == out_format_.compare("h264") ||
    0 == out_format_.compare("h265") ) {
    if (dump_output_) {
      static std::ofstream ofs(dump_output_file_ + "_" +
      std::to_string(oFrame->sp_frame_info->img_ts_.tv_sec) + "_" +
      std::to_string(oFrame->sp_frame_info->img_ts_.tv_nsec) +
      "." + out_format_);
      ofs.write(reinterpret_cast<const char*>(oFrame->mPtrData), oFrame->mDataLen);
    }

    frameh26x_sub_->index = oFrame->sp_frame_info->img_idx_;
    frameh26x_sub_->dts.sec = oFrame->sp_frame_info->img_ts_.tv_sec;
    frameh26x_sub_->dts.nanosec = oFrame->sp_frame_info->img_ts_.tv_nsec;
    frameh26x_sub_->pts.sec = oFrame->sp_frame_info->img_ts_.tv_sec;
    frameh26x_sub_->pts.nanosec = oFrame->sp_frame_info->img_ts_.tv_nsec;
    memcpy(frameh26x_sub_->encoding.data(), out_format_.c_str(), out_format_.length());
    frameh26x_sub_->width = oFrame->mWidth;
    frameh26x_sub_->height = oFrame->mHeight;

    frameh26x_sub_->data.resize(oFrame->mDataLen);
    memcpy(&frameh26x_sub_->data[0], oFrame->mPtrData, oFrame->mDataLen);
    
    ss << ", encoding: " << frameh26x_sub_->encoding.data()
      << ", w: " << frameh26x_sub_->width
      << ", h: " << frameh26x_sub_->height
      << ", size: " << frameh26x_sub_->data.size()
      << ", stamp: " << frameh26x_sub_->dts.sec
      << "." << frameh26x_sub_->dts.nanosec;

    ros_h26ximage_publisher_->publish(*frameh26x_sub_);
  } else if (0 == out_format_.compare("jpeg-compressed")) {
    compressed_img_pub_->header.stamp.sec = oFrame->sp_frame_info->img_ts_.tv_sec;
    compressed_img_pub_->header.stamp.nanosec = oFrame->sp_frame_info->img_ts_.tv_nsec;
    compressed_img_pub_->header.frame_id = "default_cam";
    compressed_img_pub_->format = "jpeg";
    // compressed_img_pub_->format = "rgb8; jpeg compressed bgr8";

    compressed_img_pub_->data.resize(oFrame->mDataLen);
    memcpy(compressed_img_pub_->data.data(), oFrame->mPtrData,
            oFrame->mDataLen);
            
    ss << ", encoding: " << compressed_img_pub_->format
      << ", size: " << compressed_img_pub_->data.size()
      << ", stamp: " << compressed_img_pub_->header.stamp.sec
      << "." << compressed_img_pub_->header.stamp.nanosec;

    if (dump_output_) {
      std::ofstream ofs(dump_output_file_ + "_" +
      std::to_string(oFrame->sp_frame_info->img_idx_) + "_" +
      std::to_string(oFrame->sp_frame_info->img_ts_.tv_sec) + "_" +
      std::to_string(oFrame->sp_frame_info->img_ts_.tv_nsec) +
      "." + out_format_);
      ofs.write(reinterpret_cast<const char*>(compressed_img_pub_->data.data()),
      compressed_img_pub_->data.size());
    }

    if (ros_compressed_image_publisher_)
      ros_compressed_image_publisher_->publish(*compressed_img_pub_);
  } else {
    img_pub_->header.stamp.sec = oFrame->sp_frame_info->img_ts_.tv_sec;
    img_pub_->header.stamp.nanosec = oFrame->sp_frame_info->img_ts_.tv_nsec;
    img_pub_->header.frame_id = "default_cam";
    img_pub_->width = oFrame->mWidth;
    img_pub_->height = oFrame->mHeight;
    img_pub_->step = oFrame->mWidth;
    if (0 == out_format_.compare("bgr8") || 0 == out_format_.compare("rgb8")) {
      img_pub_->step = oFrame->mWidth * 3;
    }
    img_pub_->encoding = out_format_.c_str();
#ifndef PLATFORM_X86
    if (CodecImgFormat::FORMAT_NV12 == oFrame->mFrameFmt) {
      if (0 == out_format_.compare("bgr8") || 0 == out_format_.compare("rgb8")) {
        int nRgbLen = oFrame->mHeight * oFrame->mWidth * 3;
        if (nullptr == mPtrOut)
          mPtrOut = new uint8_t[nRgbLen];
        if (mPtrOut) {
          if (0 == out_format_.compare("bgr8")) {
            video_utils::NV12_TO_BGR24(oFrame->mPtrY, oFrame->mPtrUV, mPtrOut, oFrame->mWidth , oFrame->mHeight);
          } else {
            video_utils::NV12_TO_RGB24(oFrame->mPtrY, oFrame->mPtrUV, mPtrOut, oFrame->mWidth , oFrame->mHeight);
          }
          img_pub_->data.resize(nRgbLen);
          memcpy(&img_pub_->data[0], mPtrOut, nRgbLen);
        }
      } else {
        int nOffSet = oFrame->mHeight * oFrame->mWidth;
        img_pub_->data.resize(oFrame->mDataLen);
        memcpy(&img_pub_->data[0], oFrame->mPtrY, nOffSet);
        memcpy(&img_pub_->data[0] + nOffSet, oFrame->mPtrUV,
          nOffSet / 2);
      }
    } else {
      img_pub_->data.resize(oFrame->mDataLen);
      memcpy(&img_pub_->data[0], oFrame->mPtrData, oFrame->mDataLen);
    }

#else
if(oFrame->mPtrData != nullptr)
{
  if (CodecImgFormat::FORMAT_BGR == oFrame->mFrameFmt) {
    int rgbLen = oFrame->mHeight * oFrame->mWidth * 3;
    if (0 == out_format_.compare("rgb8") ) {
      if (nullptr == mPtrOut) {
        mPtrOut = new uint8_t[rgbLen];
      }
      video_utils::BGR24_to_RGB24(oFrame->mPtrData, mPtrOut, oFrame->mWidth,oFrame->mHeight);
      img_pub_->data.resize(rgbLen);
      memcpy(&img_pub_->data[0], mPtrOut, rgbLen);
    } else if (0 == out_format_.compare("nv12")) {
      int nv12Len = oFrame->mHeight * oFrame->mWidth * 3 / 2;
      if (nullptr == mPtrOut) {
        mPtrOut = new uint8_t[nv12Len];
      }
      video_utils::BGR24_to_NV12(oFrame->mPtrData, mPtrOut, oFrame->mWidth, oFrame->mHeight);
      img_pub_->data.resize(nv12Len);
      memcpy(&img_pub_->data[0], mPtrOut, nv12Len);
    } else {
      img_pub_->data.resize(oFrame->mDataLen);
      memcpy(&img_pub_->data[0], oFrame->mPtrData, oFrame->mDataLen);
    }
  } else {
    img_pub_->data.resize(oFrame->mDataLen);
    memcpy(&img_pub_->data[0], oFrame->mPtrData, oFrame->mDataLen);
    
  }
}
#endif
    ss << ", encoding: " << img_pub_->encoding
    << ", w: " << img_pub_->width
    << ", h: " << img_pub_->height
    << ", step: " << img_pub_->step
    << ", size: " << img_pub_->data.size()
    << ", stamp: " << img_pub_->header.stamp.sec
    << "." << img_pub_->header.stamp.nanosec;

    if (dump_output_) {
      std::ofstream ofs(dump_output_file_ + "_" +
      std::to_string(oFrame->sp_frame_info->img_idx_) + "_" +
      std::to_string(oFrame->sp_frame_info->img_ts_.tv_sec) + "_" +
      std::to_string(oFrame->sp_frame_info->img_ts_.tv_nsec) +
      "." + out_format_);
      ofs.write(reinterpret_cast<const char*>(img_pub_->data.data()),
      img_pub_->data.size());
    }

    if (ros_image_publisher_)
      ros_image_publisher_->publish(*img_pub_);
  }

  sp_hobot_codec_impl_->ReleaseOutput(oFrame);

  auto sp_run_time_data = std::make_shared<RunTimeData>();
  sp_run_time_data->out_frame_count_ = 1;
  sp_run_time_data->out_codec_delay_ =
    tool_calc_time_laps(oFrame->sp_frame_info->img_recved_ts_,
    oFrame->sp_frame_info->img_processed_ts_);
  RunTimeStat::GetInstance()->Update(sp_run_time_data);
  auto sp_rt_data = RunTimeStat::GetInstance()->Get();
  if (sp_rt_data) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("HobotCodecNode"),
    "sub " << in_format_
    << " " << oFrame->mWidth << "x" << oFrame->mHeight
    << ", fps: " << sp_rt_data->in_frame_count_
    << ", pub " << out_format_
    << ", fps: " << sp_rt_data->out_frame_count_
    << ", comm delay: " << sp_rt_data->in_comm_delay_
    << ", codec delay: " << sp_rt_data->out_codec_delay_
    );
  }

  ss << ", codec delay ms: "
    << sp_run_time_data->out_codec_delay_;
  RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"), "%s", ss.str().data());
}

void HobotCodecNode::timer_hbmem_pub() {
  if (!rclcpp::ok()) {
    return;
  }

  if (!sp_hobot_codec_impl_) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"), "Invalid hobot codec impl");
    return;
  }

#ifdef SHARED_MEM_MSG
  auto oFrame = sp_hobot_codec_impl_->GetOutput();
  if (!oFrame) {
    if (rclcpp::ok()) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("HobotCodecNode"), "GetOutput fail!");
    }
    return;
  }

  std::stringstream ss;
  ss << "pub img"
    << ", index: " << oFrame->sp_frame_info->img_idx_;

  if (0 == out_format_.compare("h264") ||
    0 == out_format_.compare("h265") ) {
    if (dump_output_) {
      static std::ofstream ofs(dump_output_file_ + "_" +
      std::to_string(oFrame->sp_frame_info->img_idx_) + "_" +
      std::to_string(oFrame->sp_frame_info->img_ts_.tv_sec) + "_" +
      std::to_string(oFrame->sp_frame_info->img_ts_.tv_nsec) +
      "." + out_format_);
      ofs.write(reinterpret_cast<const char*>(oFrame->mPtrData), oFrame->mDataLen);
    }

    if (!h264hbmem_publisher_) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("HobotCodecNode"), "Invalid h264hbmem_publisher_!");
      return;
    }
    auto loanedMsg = h264hbmem_publisher_->borrow_loaned_message();
    if (loanedMsg.is_valid()) {
      auto& msg = loanedMsg.get();
      msg.dts.sec = oFrame->sp_frame_info->img_ts_.tv_sec;
      msg.dts.nanosec = oFrame->sp_frame_info->img_ts_.tv_nsec;
      msg.pts.sec = oFrame->sp_frame_info->img_ts_.tv_sec;
      msg.pts.nanosec = oFrame->sp_frame_info->img_ts_.tv_nsec;
      msg.data_size = oFrame->mDataLen;
      msg.height = oFrame->mHeight;
      msg.width = oFrame->mWidth;
      memcpy(msg.encoding.data(), out_format_.c_str(), out_format_.length());
      memcpy(msg.data.data(), oFrame->mPtrData, oFrame->mDataLen);
      msg.index = mSendIdx++;
      
      ss << ", encoding: " << msg.encoding.data()
      << ", w: " << msg.width
      << ", h: " << msg.height
      << ", size: " << msg.data.size()
      << ", stamp: " << msg.dts.sec
      << "." << msg.dts.nanosec;

      h264hbmem_publisher_->publish(std::move(loanedMsg));
    } else {
      RCLCPP_WARN(rclcpp::get_logger("HobotCodecNode"), "hbm_h26x borrow_loaned_message failed");
    }
  } else {
    if (!hbmem_publisher_) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("HobotCodecNode"), "Invalid hbmem_publisher_!");
      return;
    }
    auto loanedMsg = hbmem_publisher_->borrow_loaned_message();
    if (loanedMsg.is_valid()) {
      auto& msg = loanedMsg.get();
      msg.time_stamp.sec = oFrame->sp_frame_info->img_ts_.tv_sec;
      msg.time_stamp.nanosec = oFrame->sp_frame_info->img_ts_.tv_nsec;
      memcpy(msg.encoding.data(), out_format_.c_str(), out_format_.length());
      msg.height = oFrame->mHeight;
      msg.width = oFrame->mWidth;
      msg.step = oFrame->mWidth;
      if (0 == out_format_.compare("bgr8") || 0 == out_format_.compare("rgb8")) {
        img_pub_->step = oFrame->mWidth * 3;
      }
      msg.data_size = oFrame->mDataLen;
      int nOffSet = oFrame->mHeight * oFrame->mWidth;
      RCLCPP_DEBUG(rclcpp::get_logger("HobotCodecNode"), "mFrameFmt: %d",
      static_cast<int>(oFrame->mFrameFmt));
#ifndef PLATFORM_X86
      if (CodecImgFormat::FORMAT_INVALID == oFrame->mFrameFmt) {
        RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"), "Invalid mFrameFmt: %d",
        static_cast<int>(oFrame->mFrameFmt));
        return;
      } else if (CodecImgFormat::FORMAT_NV12 == oFrame->mFrameFmt) {
        if (0 == out_format_.compare("bgr8") || 0 == out_format_.compare("rgb8")) {
          int nRgbLen = oFrame->mHeight * oFrame->mWidth * 3;
          msg.data_size = nRgbLen;
          if (nullptr == mPtrOut)
            mPtrOut = new uint8_t[nRgbLen];
          if (mPtrOut) {
            if (0 == out_format_.compare("bgr8")) {
              video_utils::NV12_TO_BGR24(oFrame->mPtrY, oFrame->mPtrUV, mPtrOut, oFrame->mWidth , oFrame->mHeight);
            } else {
              video_utils::NV12_TO_RGB24(oFrame->mPtrY, oFrame->mPtrUV, mPtrOut, oFrame->mWidth , oFrame->mHeight);
            }
            memcpy(msg.data.data(), mPtrOut, nRgbLen);
          }
        } else {
          memcpy(msg.data.data(), oFrame->mPtrY, nOffSet);
          memcpy(msg.data.data() + nOffSet, oFrame->mPtrUV,
            nOffSet / 2);
        }
      } else {
        memcpy(msg.data.data(), oFrame->mPtrData, oFrame->mDataLen);
      }
#else
if(oFrame->mPtrData != nullptr)
{
  if (CodecImgFormat::FORMAT_INVALID == oFrame->mFrameFmt) {
  RCLCPP_ERROR(rclcpp::get_logger("HobotCodecNode"), "Invalid mFrameFmt: %d",
  static_cast<int>(oFrame->mFrameFmt));
  return;
  } else if (CodecImgFormat::FORMAT_BGR == oFrame->mFrameFmt) {
    int rgbLen = oFrame->mHeight * oFrame->mWidth * 3;
    if (0 == out_format_.compare("rgb8") ) {
      if (nullptr == mPtrOut) {
        mPtrOut = new uint8_t[rgbLen];
      }
      video_utils::BGR24_to_RGB24(oFrame->mPtrData, mPtrOut, oFrame->mWidth,oFrame->mHeight);
      msg.data_size = rgbLen;
      memcpy(msg.data.data(), mPtrOut, rgbLen);
    } else if (0 == out_format_.compare("nv12")) {
      int nv12Len = oFrame->mHeight * oFrame->mWidth * 3 / 2;
      if (nullptr == mPtrOut) {
        mPtrOut = new uint8_t[nv12Len];
      }
      video_utils::BGR24_to_NV12(oFrame->mPtrData, mPtrOut, oFrame->mWidth, oFrame->mHeight);
      msg.data_size = nv12Len;
      memcpy(msg.data.data(), mPtrOut, nv12Len);
    } else {
      msg.data_size = oFrame->mDataLen;
      memcpy(msg.data.data(), oFrame->mPtrData, oFrame->mDataLen);
    }
  } else {
    msg.data_size = oFrame->mDataLen;
    memcpy(msg.data.data(), oFrame->mPtrData, oFrame->mDataLen);
  }
}
#endif
      msg.index = mSendIdx++;
      
      ss << ", encoding: " << msg.encoding.data()
      << ", w: " << msg.width
      << ", h: " << msg.height
      << ", step: " << msg.step
      << ", size: " << msg.data_size
      << ", stamp: " << msg.time_stamp.sec
      << "." << msg.time_stamp.nanosec;

      if (dump_output_) {
        std::ofstream ofs(dump_output_file_ + "_" +
        std::to_string(oFrame->sp_frame_info->img_idx_) + "_" +
        std::to_string(oFrame->sp_frame_info->img_ts_.tv_sec) + "_" +
        std::to_string(oFrame->sp_frame_info->img_ts_.tv_nsec) +
        "." + out_format_);
        ofs.write(reinterpret_cast<const char*>(msg.data.data()), msg.data_size);
      }

      hbmem_publisher_->publish(std::move(loanedMsg));
    } else {
      RCLCPP_WARN(rclcpp::get_logger("HobotCodecNode"), "borrow_loaned_message failed");
    }
  }
  sp_hobot_codec_impl_->ReleaseOutput(oFrame);
  
  auto sp_run_time_data = std::make_shared<RunTimeData>();
  sp_run_time_data->out_frame_count_ = 1;
  sp_run_time_data->out_codec_delay_ =
    tool_calc_time_laps(oFrame->sp_frame_info->img_recved_ts_,
    oFrame->sp_frame_info->img_processed_ts_);
  RunTimeStat::GetInstance()->Update(sp_run_time_data);
  auto sp_rt_data = RunTimeStat::GetInstance()->Get();
  if (sp_rt_data) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("HobotCodecNode"),
    "sub " << in_format_
    << " " << oFrame->mWidth << "x" << oFrame->mHeight
    << ", fps: " << sp_rt_data->in_frame_count_
    << ", pub " << out_format_
    << ", fps: " << sp_rt_data->out_frame_count_
    << ", comm delay: " << sp_rt_data->in_comm_delay_
    << ", codec delay: " << sp_rt_data->out_codec_delay_
    );
  }
    
  ss << ", codec delay ms: "
    << sp_run_time_data->out_codec_delay_;
  RCLCPP_INFO(rclcpp::get_logger("HobotCodecNode"), "%s", ss.str().data());
#endif
}

int RunTimeStat::Update(std::shared_ptr<RunTimeData> sp_run_time_data) {
  if (!sp_run_time_data) {
    return -1;
  }
  std::unique_lock<std::mutex> lk(frame_stat_mtx_);
  if (!last_frame_tp_) {
    last_frame_tp_ =
        std::make_shared<std::chrono::high_resolution_clock::time_point>();
    *last_frame_tp_ = std::chrono::system_clock::now();
  }

  run_time_data_.in_frame_count_ += sp_run_time_data->in_frame_count_;
  run_time_data_.in_comm_delay_ += sp_run_time_data->in_comm_delay_;
  run_time_data_.in_codec_delay_ += sp_run_time_data->in_codec_delay_;
  run_time_data_.out_frame_count_ += sp_run_time_data->out_frame_count_;
  run_time_data_.out_codec_delay_ += sp_run_time_data->out_codec_delay_;

  return 0;
}

std::shared_ptr<RunTimeData> RunTimeStat::Get(int time_interval_ms) {
  std::unique_lock<std::mutex> lk(frame_stat_mtx_);
  if (!last_frame_tp_ || time_interval_ms <= 0) {
    return nullptr;
  }

  auto tp_now = std::chrono::system_clock::now();
  auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                      tp_now - *last_frame_tp_)
                      .count();
  if (interval >= time_interval_ms) {
    if (run_time_data_.in_frame_count_ <= 0 || run_time_data_.out_frame_count_ <= 0) {
      return nullptr;
    }

    auto run_time_data = std::make_shared<RunTimeData>();
    // cal the average time delay with frame count
    run_time_data->in_comm_delay_ = run_time_data_.in_comm_delay_ / run_time_data_.in_frame_count_;
    run_time_data->in_codec_delay_ = run_time_data_.in_codec_delay_ / run_time_data_.in_frame_count_;

    run_time_data->out_codec_delay_ = run_time_data_.out_codec_delay_ / run_time_data_.out_frame_count_;

    // update frame count as frame fps
    run_time_data->in_frame_count_ = run_time_data_.in_frame_count_ /
                (static_cast<float>(interval) / 1000.0);
    run_time_data->out_frame_count_ = run_time_data_.out_frame_count_ /
                (static_cast<float>(interval) / 1000.0);
    
    memset(&run_time_data_, 0, sizeof(RunTimeData));
    *last_frame_tp_ = std::chrono::system_clock::now();
    return run_time_data;
  }
  return nullptr;
}
