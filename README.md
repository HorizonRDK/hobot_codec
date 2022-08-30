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

支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### X3 Ubuntu系统板端编译
1、编译环境确认

 - 板端已安装X3 Ubuntu系统。
 - 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
 - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`

2、编译：
  - `colcon build --packages-select hobot_codec`

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
     -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
  ```
- 其中SYS_ROOT为交叉编译系统依赖路径，此路径具体地址详见第1步“编译环境确认”的交叉编译说明。

# Usage

## X3 Ubuntu系统
编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行

```
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
ros2 run hobot_codec hobot_codec_republish
```
### 目前参数列表：

| 参数名           | 含义                         | 取值                                          | 默认值                |
| ---------------- | ---------------------------- | --------------------------------------------- | --------------------- |
| channel          | 处理通道号                   | 0-3                                           | 0                     |
| in_mode          | 接入数据传输的方式           | ros/shared_mem                                | ros                   |
| out_mode         | 发出数据传输方式             | ros/shared_mem                                | ros                   |
| in_format        | 订阅的数据格式               | bgr8/rgb8/nv12/jpeg/h264/h265                 | bgr8                  |
| out_format       | 处理后发布的数据格式         | bgr8/rgb8/nv12/jpeg/jpeg-compressed/h264/h265 | jpeg                  |
| sub_topic        | 订阅的话题名字               | 任意字符串，但必须是别的node 发布的topic      | /image_raw            |
| pub_topic        | 发布的话题名字               | 任意字符串                                    | /image_raw/compressed |
| enc_qp           | 264/265编码质量              | 浮点数 0-100                                  | 10.0                  |
| jpg_quality      | jpeg 编码质量                | 浮点数 0-100                                  | 60.0                  |
| input_framerate  | 输入帧率，实际送数据帧率     | 正整数                                        | 30                    |
| output_framerate | 输出帧率，仅编码模式支持配置 | 正整数                                        |                       |

### 注意：

    由于编码器限制，目前可知 jpeg/h264/h265 960*540 不支持， 960*544 可以支持，720P/D1/VGA 系列部分支持。

运行方式1，命令方式：
解码 jpeg 测试：
```
ros2 run mipi_cam mipi_cam --ros-args -p video_device:=F37 -p image_width:=640 -p image_height:=480 -p out_format:=nv12
// 先用获取的 mipi_cam 的数据进行 jpeg 编码
ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=ros -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/image_raw -p pub_topic:=/image_jpeg

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=ros -p in_format:=jpeg -p out_mode:=ros -p out_format:=rgb8 -p sub_topic:=/image_jpeg -p pub_topic:=/image_raw_nv12
```
编码 jpeg 测试：
```
ros2 run mipi_cam mipi_cam --ros-args -p video_device:=F37 -p image_width:=640 -p image_height:=480 -p out_format:=nv12

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=ros -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/image_raw -p pub_topic:=/image_jpeg

#share memory
ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=0 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/hbmem_img -p pub_topic:=/image_jpeg
```
编码jpeg，发布compressed格式消息，并设置输出帧率：

~~~shell
ros2 run mipi_cam mipi_cam --ros-args -p video_device:=F37 -p image_width:=640 -p image_height:=480 -p out_format:=nv12

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=ros -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg-compressed -p sub_topic:=/image_raw -p pub_topic:=/image_jpeg/compressed -p output_framerate:=20
~~~

编解码 h264 测试：

```
ros2 run mipi_cam mipi_cam --ros-args -p out_format:=nv12

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=0 -p in_mode:=ros -p in_format:=nv12 -p out_mode:=ros -p out_format:=h264 -p sub_topic:=/image_raw -p pub_topic:=/image_h264

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=0 -p in_mode:=ros -p in_format:=h264 -p out_mode:=ros -p out_format:=nv12 -p sub_topic:=/image_h264 -p pub_topic:=/image_nv12
```
运行方式2，使用launch文件启动：
`ros2 launch install/share/hobot_codec/launch/hobot_codec.launch.py`


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


## 高级使用范例：

串联（前一个结点 hobot_codec_republish 为 编码，后面一个 hobot_codec_republish sub 前一个codec 节点的 pub 数据进行解码）测试编解码：
```
export ROS_DOMAIN_ID=***
# 配置 ROS_DOMAIN_ID，避免多机干扰，每一个 terminal 都需要执行，才可以进行 sub 到数据

ros2 run mipi_cam mipi_cam --ros-args --log-level info --ros-args -p out_format:=nv12 -p io_method:=shared_mem -p image_width:=640 -p image_height:=480
# 运行mipi cam，通过shared mem 方式发布nv12格式图片

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=shared_mem -p out_format:=h265 -p sub_topic:=/hbmem_img -p pub_topic:=/image_h265
# 运行codec，订阅nv12格式图片，编码并发布h265格式视频，话题：image_h265，方式 shared_mem

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=h265 -p out_mode:=shared_mem -p out_format:=nv12 -p sub_topic:=/image_h265 -p pub_topic:=/image_nv12
# 运行codec，订阅h265格式视频（通过shared_mem方式，话题必须是：image_h265），解码并发布nv12格式图片

```