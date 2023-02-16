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
#include <list>
#include <thread>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "include/hobot_codec_data.h"
#include "include/hobot_codec_base.h"

int HobotCodecBase::CheckParams(const std::shared_ptr<HobotCodecParaBase>& sp_hobot_codec_para) {
  if (!sp_hobot_codec_para) {
    RCLCPP_ERROR(rclcpp::get_logger("HobotCodecBase"), "Invalid input");
    return -1;
  }
  return 0;
}
  