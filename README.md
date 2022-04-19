Getting Started with hobot_codec
=======

# Intro

hobot_codec package用于显示接收 ROS2 Node 发布的image msg。支持ROS标准格式，也支持 share mem 方式订阅，发布 jpg/h264/h265 话题

# Build
---
## Dependency

ros package：
- sensor_msgs
- hbm_img_msgs

其中cv_bridge为ROS开源的package，需要手动安装，具体安装方法：

```cpp
# 方法1，直接使用apt安装，以cv_bridge安装举例
sudo apt-get install ros-foxy-cv-bridge -y

# 方法2，使用rosdep检查并自动安装pkg编译的依赖项
# 安装ros pkg依赖下载⼯具rosdep
sudo apt-get install python3-pip
sudo pip install rosdep
sudo rosdep init
rosdep update
# 在ros的⼯程路径下执⾏安装依赖，需要指定pkg所在路径。默认为所有pkg安装依赖，也可以指定为某个pkg安装依赖
rosdep install -i --from-path . --rosdistro foxy -y
```

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式，并支持通过编译选项控制编译pkg的依赖和pkg的功能。
### 编译选项

SHARED_MEM

- shared mem（共享内存传输）使能开关，默认关闭（OFF），编译时使用-DSHARED_MEM=ON命令打开。
- 如果打开，编译和运行会依赖hbm_img_msgs pkg，并且需要使用tros进行编译。
- 如果关闭，编译和运行不依赖hbm_img_msgs pkg，支持使用原生ros和tros进行编译。
- CMakeLists.txt中指定 hobot_codec package的安装路径，默认为`../install/hobot_codec`。

### X3 Ubuntu系统上编译
1、编译环境确认

- 当前编译终端已设置ROS环境变量：`source /opt/ros/foxy/setup.bash`。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon，需要手动安装colcon。colcon安装命令：`apt update; apt install python3-colcon-common-extensions`
- 已依赖pkg ，详见 Dependency 部分

2、编译：
  - 支持 share mem 方式订阅发布的图片：`colcon build --packages-select hobot_codec --cmake-args -DSHARED_MEM=ON`
  - 支持订阅ROS2标准格式图片：`colcon build --packages-select hobot_codec`或`colcon build --packages-select hobot_codec --cmake-args -DSHARED_MEM=OFF`。

### docker交叉编译

1、编译环境确认

- 在docker中编译，并且docker中已经安装好tros。docker安装、交叉编译说明、tros编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。
- 已编译hbm_img_msgs package（编译方法见Dependency部分）

2、编译

- 编译命令： 

  ```
  export TARGET_ARCH=aarch64
  export TARGET_TRIPLE=aarch64-linux-gnu
  export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-
  
  colcon build --packages-select hobot_codec \
     --merge-install \
     --cmake-force-configure \
     --cmake-args \
     --no-warn-unused-cli \
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake \
     -DSHARED_MEM=ON
  ```
- 其中SYS_ROOT为交叉编译系统依赖路径，此路径具体地址详见第1步“编译环境确认”的交叉编译说明。

# Usage

## X3 Ubuntu系统
编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
ros2 run hobot_codec hobot_codec_republish
# 目前参数列表：

参数名 | 含义 | 取值 | 默认值
---|---|---|---
channel     | 处理通道号          | 0-3                            | 0
in_mode     | 接入数据传输的方式   | ros/shared_mem                 | ros
out_mode    | 发出数据传输方式     | ros/shared_mem                 | ros
in_format   | 订阅的数据格式       | bgr8/rgb8/nv12/jpeg/h264/h265  | bgr8
out_format  | 处理后发布的数据格式 | bgr8/rgb8/nv12/jpeg/h264/h265   |jpeg
sub_topic   | 订阅的话题名字       | 任意字符串                      | /image_raw
pub_topic   | 发布的话题名字       | 任意字符串                      | /image_raw/compressed
enc_qp      | 264/265编码质量     | 浮点数 0-100                    | 10.0
jpg_quality | jpeg 编码质量       | 浮点数 0-100                    | 60.0


解码 jpeg 测试：
```
ros2 run mipi_cam mipi_cam --ros-args -p video_device:=F37 -p image_width:=960 -p image_height:=540

ros2 run image_transport republish raw compressed --ros-args --remap in:=/image_raw --remap out/compressed:=/image_raw/compressed

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=ros -p in_format:=jpeg -p out_mode:=ros -p out_format:=rgb8 -p sub_topic:=/image_raw/compressed -p pub_topic:=/image_raw_nv12
```
编码 jpeg 测试：
ros2 run mipi_cam mipi_cam --ros-args -p video_device:=F37 -p image_width:=960 -p image_height:=540 -p out_format:=nv12

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=ros -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/image_raw -p pub_topic:=/image_jpeg

share memory
ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=0 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg


```

## X3 linaro系统

把在docker 交叉编译的install 目录拷贝到linaro 系统下，例如:/userdata
需要首先指定依赖库的路径，例如：
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/userdata/install/lib`

修改 ROS_LOG_DIR 的路径，否则会创建在 /home 目录下，需要执行 mount -o remount,rw /，才可以在 /home 下创建日志
`export ROS_LOG_DIR=/userdata/`

运行 hobot_codec_republish
```
// 默认参数方式
/userdata/install/lib/hobot_codec/hobot_codec_republish
// 传参方式
#/userdata/install/lib/hobot_codec/hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=ros -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/image_raw -p pub_topic:=/image_jpeg

```
