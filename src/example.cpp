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

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
