# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(hobot_codec)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# BUILD_HBMEM is set in aarch64_toolchainfile.cmake
if (${BUILD_HBMEM})
  message("using shared mem")
  find_package(hbm_img_msgs REQUIRED)
  add_definitions(-DSHARED_MEM_MSG)
else()
  message("not using shared mem")
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(img_msgs REQUIRED)

# x3|j5|x86
set(PREFIX_PATH x3)

if(PLATFORM_X3)
  message("build platform X3")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)
elseif(PLATFORM_J5)
  message("build platform J5")
  add_definitions(-DPLATFORM_J5)
  set(PREFIX_PATH j5)
elseif(PLATFORM_X86)
  message("build platform X86")
  add_definitions(-DPLATFORM_X86)
  set(PREFIX_PATH x86)
  message("THIRD_PARTY is " ${THIRD_PARTY})
  find_package(OpenCV 4 REQUIRED)
else()
  message("invalid platform, build platform X3 default")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)
endif()

message("PREFIX_PATH: ${PREFIX_PATH}")

include_directories(include
  include/encoder/${PREFIX_PATH}
  include/decoder/${PREFIX_PATH}
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_SYSROOT}/usr/include
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbmedia
)

add_executable(hobot_codec_republish
  src/example.cpp
  src/hobot_codec_node.cpp
  src/hobot_codec_impl.cpp
  src/hobot_codec_base.cpp
  src/decoder/${PREFIX_PATH}/hobot_vdec.cpp
  src/encoder/${PREFIX_PATH}/hobot_venc.cpp
)

set(BASE_LIBRARIES pthread dl)
# 根据平台选择需要链接的库
if(PLATFORM_X3)
  set(SYS_LIB vio isp_algo z hbmedia isp ion multimedia iar tinyalsa cjson)
elseif(PLATFORM_J5)
  set(SYS_LIB multimedia)
elseif(PLATFORM_X86)
  set(SYS_LIB ${OpenCV_LIBS})
else()
  message("link platform X3 libs default")
  set(SYS_LIB vio isp_algo z hbmedia isp ion multimedia iar tinyalsa cjson)
endif()

#${PROJECT_NAME}
target_link_libraries(hobot_codec_republish
  ${BASE_LIBRARIES} ${SYS_LIB}
)

if (${BUILD_HBMEM})
ament_target_dependencies(
  hobot_codec_republish
  rclcpp
  sensor_msgs
  img_msgs
  hbm_img_msgs
)
else()
ament_target_dependencies(
  hobot_codec_republish
  rclcpp
  img_msgs
  sensor_msgs
)
endif()

target_link_libraries(hobot_codec_republish
)

# Install executables
install(
  TARGETS hobot_codec_republish
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
  DESTINATION lib/${PROJECT_NAME}/config/
)

ament_package()
