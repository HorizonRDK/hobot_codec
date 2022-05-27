// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "include/hobot_codec.h"

// ros2 run hobot_codec hobot_codec_republish ros jpeg decompress sub_topic:=/image_raw/compressed
// pub_topic:=/image_raw rgb8
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_WARN(rclcpp::get_logger("example"), "This is HobotCodec example!");
  rclcpp::NodeOptions opt;

  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  static rclcpp::Time start = ros_clock.now();
  std::string strName = "hobot_codec";

  // auto node = std::make_shared<HobotCodec>(opt, "hobotCodec", argv[1], argv[2], argv[3], argv[4]);
  auto node = std::make_shared<HobotCodec>(opt, strName + std::to_string(start.nanoseconds()));
  RCLCPP_WARN(rclcpp::get_logger("example"), "HobotCodec init!");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  RCLCPP_WARN(rclcpp::get_logger("example"), "HobotCodec add_node!");
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
