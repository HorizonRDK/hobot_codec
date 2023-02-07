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
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "include/hobot_codec_node.h"


// ros2 run hobot_codec hobot_codec_republish ros jpeg decompress sub_topic:=/image_raw/compressed
// pub_topic:=/image_raw rgb8
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::string node_name = "hobot_codec_" + std::to_string(getpid());
  RCLCPP_WARN(rclcpp::get_logger("HobotCodec"), "This is HobotCodecNode: %s.", node_name.data());
  rclcpp::spin(std::make_shared<HobotCodecNode>(rclcpp::NodeOptions(), node_name));
  rclcpp::shutdown();
  RCLCPP_WARN(rclcpp::get_logger("HobotCodec"), "HobotCodecNode: %s exit.", node_name.data());
  return 0;
}
