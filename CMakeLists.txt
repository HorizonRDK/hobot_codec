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
  add_definitions(-DUSING_HBMEM)
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

include_directories(include
${PROJECT_SOURCE_DIR}
${CMAKE_SYSROOT}/usr/include
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbmedia
)

add_executable(hobot_codec_republish
src/example.cpp
src/hobot_codec.cpp
src/hobot_vdec.cpp
src/hobot_venc.cpp
)

set(BASE_LIBRARIES pthread vio dl)
set(X3_HAPI_LIB isp_algo z hbmedia isp ion multimedia iar tinyalsa cjson)

#${PROJECT_NAME}
target_link_libraries(hobot_codec_republish
  ${BASE_LIBRARIES} ${X3_HAPI_LIB}
)

if (${BUILD_HBMEM})
target_link_libraries(hobot_codec_republish
  hbmem alog
)
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

ament_package()
