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
- 系统版本：Ubuntu 20.04
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### X3 Ubuntu系统板端编译X3版本

1、编译环境确认

- 板端已安装X3 Ubuntu系统。
- 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
- 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`

2、编译：

- `colcon build --packages-select hobot_codec --cmake-args -DBUILD_HBMEM=ON`，使用编译选项`BUILD_HBMEM`使能了零拷贝数据传输功能。

### docker交叉编译X3版本

1、编译环境确认

- 在docker中编译，并且docker中已经安装好tros。docker安装、交叉编译说明、tros编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。
- 已编译hbm_img_msgs package（编译方法见Dependency部分）

2、编译

- 编译命令：

  ```shell
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

### X86 Ubuntu系统上编译 X86版本

1、编译环境确认

- X86 Ubuntu版本：Ubuntu20.04
- Opencv：4.2.0

2、编译

- 编译命令：

  ```shell
  colcon build --packages-select hobot_codec  \
     --merge-install \
     --cmake-args \
     -DPLATFORM_X86=ON \
     -DBUILD_HBMEM=ON \
     -DTHIRD_PARTY=`pwd`/../sysroot_docker
  ```

# Usage

## X3 Ubuntu系统

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
ros2 run hobot_codec hobot_codec_republish
```

### 目前参数列表

| 参数名           | 含义                         | 取值                                          | 默认值                |
| ---------------- | ---------------------------- | --------------------------------------------- | --------------------- |
| channel          | 处理通道号                   | 0-3                                           | 0                     |
| in_mode          | 接入数据传输的方式           | ros/shared_mem                                | ros                   |
| out_mode         | 发出数据传输方式             | ros/shared_mem                                | ros                   |
| in_format        | 订阅的数据格式               | bgr8/rgb8/nv12/jpeg/jpeg-compressed/h264/h265 | bgr8                  |
| out_format       | 处理后发布的数据格式         | bgr8/rgb8/nv12/jpeg/jpeg-compressed/h264/h265 | jpeg                  |
| sub_topic        | 订阅的话题名字               | 任意字符串，但必须是别的node 发布的topic      | /image_raw            |
| pub_topic        | 发布的话题名字               | 任意字符串                                    | /image_raw/compressed |
| enc_qp           | 264/265编码质量              | 浮点数 0-100                                  | 10.0                  |
| jpg_quality      | jpeg 编码质量                | 浮点数 0-100                                  | 60.0                  |
| input_framerate  | 输入帧率，实际送数据帧率     | 正整数                                        | 30                    |
| output_framerate | 输出帧率，仅编码模式支持配置 | 正整数，小于等于输入帧率                      | -1（不开启帧率控制）  |
| dump_output      | 存储编解码输出配置            | True存储，False不存储                          | False                 |

### 注意

    1、编解码要求图像分辨率的width和height都是8字节对齐。
    2、X86版本仅支持bgr8/rgb8/nv12与jpeg的相互转换。
    3、当对h264、h265进行解码时，hobot_codec需要从视频第一帧开始解析。
    4、对于非零拷贝数据传输方式（in_mode/out_mode参数取值为ros），当`in_format/out_format`参数取值为`jpeg-compressed`时，订阅/发布的话题数据类型为`sensor_msgs::msg::CompressedImage`；参数取值为`bgr8/rgb8/nv12/jpeg`时，订阅/发布的话题数据类型为`sensor_msgs::msg::Image`。


### 运行方式1，命令方式

1. 订阅MIPI摄像头发布的NV12格式图片，编码成JPEG图片并存储编码后的图片：

~~~shell
# MIPI摄像头通过零拷贝发布NV12格式图片
ros2 run mipi_cam mipi_cam --ros-args -p out_format:=nv12 -p io_method:=shared_mem --log-level info

# 编码成JPEG
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=shared_mem -p out_format:=jpeg -p sub_topic:=/hbmem_img -p dump_output:=True
~~~

2. 订阅图像发布工具发布的NV12格式图片，测试编码：

~~~shell
# 图像发布工具发布NV12格式图片
cp -r /opt/tros/lib/hobot_image_publisher/config/ .
ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/test1.jpg -p output_image_w:=960 -p output_image_h:=544 -p image_format:=jpg -p source_image_w:=960 -p source_image_h:=544

# 编码成jpeg
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=shared_mem -p out_format:=jpeg -p sub_topic:=/hbmem_img -p dump_output:=False

# 编码成h264
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=shared_mem -p out_format:=h264 -p sub_topic:=/hbmem_img -p dump_output:=False

# 编码成h265
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=shared_mem -p out_format:=h265 -p sub_topic:=/hbmem_img -p dump_output:=False
~~~

2. 订阅H264视频，解码出NV12格式图片：

~~~shell
# 解码成nv12图片
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=h264 -p out_mode:=shared_mem -p out_format:=nv12 -p sub_topic:=/hbmem_img -p dump_output:=False

# 图像发布工具发布h264视频
cp -r /opt/tros/lib/hobot_image_publisher/config/ .
ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/test1.h264 -p image_format:=h264
~~~

3. 订阅H265视频，解码出NV12格式图片：

~~~shell
# 解码成nv12图片
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=h265 -p out_mode:=shared_mem -p out_format:=nv12 -p sub_topic:=/hbmem_img -p dump_output:=False

# 图像发布工具发布h265视频
cp -r /opt/tros/lib/hobot_image_publisher/config/ .
ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/sky.h265 -p image_format:=h265
~~~

4. 订阅JPEG图片后解码成NV12格式：

~~~shell
# 订阅jpeg图片，解码成nv12图片
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=jpeg -p out_mode:=shared_mem -p out_format:=nv12 -p sub_topic:=/image_jpeg -p dump_output:=False

# 图像发布工具发布JPEG格式图片
cp -r /opt/tros/lib/hobot_image_publisher/config/ .
ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/test1.jpg -p output_image_w:=960 -p output_image_h:=544 -p image_format:=jpg -p is_compressed_img_pub:=True -p msg_pub_topic_name:=/image_jpeg
~~~

### 运行方式2，使用launch文件启动

```shell
source /opt/tros/setup.bash

ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37
```

```shell
source /opt/tros/setup.bash

ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/hbmem_img codec_pub_topic:=/image_jpeg
```

## X3 linaro系统

把在docker 交叉编译的install 目录拷贝到linaro 系统下，例如:/userdata
需要首先指定依赖库的路径，例如：
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/userdata/install/lib`

修改 ROS_LOG_DIR 的路径，否则会创建在 /home 目录下，需要执行 mount -o remount,rw /，才可以在 /home 下创建日志
`export ROS_LOG_DIR=/userdata/`

运行 hobot_codec_republish

```shell
// 默认参数方式
/userdata/install/lib/hobot_codec/hobot_codec_republish
// 传参方式
#/userdata/install/lib/hobot_codec/hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=ros -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/image_raw -p pub_topic:=/image_jpeg

```

## X86 Ubuntu系统

编译成功后，执行如下命令运行

```shell
source ./install/local_setup.sh

# 图像发布工具发布NV12格式图片
cp -r /opt/tros/lib/hobot_image_publisher/config/ .
ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/test1.jpg -p output_image_w:=960 -p output_image_h:=544 -p image_format:=jpg -p source_image_w:=960 -p source_image_h:=544

# 编码成jpeg
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=shared_mem -p out_format:=jpeg -p sub_topic:=/hbmem_img -p dump_output:=False
```

## 高级使用范例

串联（前一个结点 hobot_codec_republish 为 编码，后面一个 hobot_codec_republish sub 前一个codec 节点的 pub 数据进行解码）测试编解码：

```shell
export ROS_DOMAIN_ID=***
# 配置 ROS_DOMAIN_ID，避免多机干扰，每一个 terminal 都需要执行，才可以进行 sub 到数据

ros2 run mipi_cam mipi_cam --ros-args --log-level info --ros-args -p out_format:=nv12 -p io_method:=shared_mem -p image_width:=640 -p image_height:=480
# 运行mipi cam，通过shared mem 方式发布nv12格式图片

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=shared_mem -p out_format:=h265 -p sub_topic:=/hbmem_img -p pub_topic:=/image_h265
# 运行codec，订阅nv12格式图片，编码并发布h265格式视频，话题：image_h265，方式 shared_mem

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=h265 -p out_mode:=shared_mem -p out_format:=nv12 -p sub_topic:=/image_h265 -p pub_topic:=/image_nv12
# 运行codec，订阅h265格式视频（通过shared_mem方式，话题必须是：image_h265），解码并发布nv12格式图片

```

## 性能展示

1、测试方法

- 硬件：RDK X3 4G版本，RDK Ultra开发板

- 锁定CPU频率：`echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor`

- CPU占用为使用top命令查看测试进程的单核CPU占比。

- 耗时表示从订阅到数据到完成编码/编码后发布数据的时间间隔，统计单位为ms，取平均值。

- 使用图像发布工具hobot_image_publisher通过零拷贝发布图片/视频，测试编码/编码，分辨率1920x1080

- 测试命令

| 类型  | 输入格式 | 输出格式 | 执行命令 |
| ----  | ------- | ------- | ------- |
| 解码  |  JPEG   |  NV12   | ros2 launch hobot_codec hobot_codec_benchmark.launch.py image_source:=1920x1080.jpeg image_format:=jpeg codec_in_format:=jpeg codec_out_format:=nv12 |
| 解码  |  H264   |  NV12   | ros2 launch hobot_codec hobot_codec_benchmark.launch.py image_source:=1920x1080.h264 image_format:=h264 codec_in_format:=h264 codec_out_format:=nv12 |
| 解码  |  H265   |  NV12   | ros2 launch hobot_codec hobot_codec_benchmark.launch.py image_source:=1920x1080.h265 image_format:=h265 codec_in_format:=h265 codec_out_format:=nv12 |
| 编码  |  NV12   |  JPEG   | ros2 launch hobot_codec hobot_codec_benchmark.launch.py image_source:=1920x1080.nv12 image_format:=nv12 codec_in_format:=nv12 codec_out_format:=jpeg |
| 编码  |  NV12   |  H264   | ros2 launch hobot_codec hobot_codec_benchmark.launch.py image_source:=1920x1080.nv12 image_format:=nv12 codec_in_format:=nv12 codec_out_format:=h264 |
| 编码  |  NV12   |  H265   | ros2 launch hobot_codec hobot_codec_benchmark.launch.py image_source:=1920x1080.nv12 image_format:=nv12 codec_in_format:=nv12 codec_out_format:=h265 |

2、测试结果

X3派：

| 类型  | 输入格式 | 输出格式 | 耗时 | CPU占用 | 内存占用 | 输入帧率 | 输出帧率 |
| ----  | ------- | ------- | ---- | ------ | -------- | ------- | ------- |
| 解码  |  JPEG   |  NV12   | 4.4  | 17.0%   |  0.7%   |  30.3   |   30.3  |
| 解码  |  H264   |  NV12   | 88.3 | 9.7%    |  0.7%   |  24.1   |   24.1  |
| 解码  |  H265   |  NV12   | 87.0 | 9.6%    |  0.7%   |  24.1   |   24.1  |
| 编码  |  NV12   |  JPEG   | 4.7  | 13.0%   |  0.8%   |  30.3   |   30.3  |
| 编码  |  NV12   |  H264   | 5.5  | 11.0%   |  0.8%   |  30.3   |   30.3  |
| 编码  |  NV12   |  H265   | 5.7  | 11.0%   |  0.8%   |  30.3   |   30.3  |

RDK Ultra

| 类型  | 输入格式 | 输出格式 | 耗时 | CPU占用 | 内存占用 | 输入帧率 | 输出帧率 |
| ----  | ------- | ------- | ---- | ------ | -------- | ------- | ------- |
| 解码  |  JPEG   |  NV12   | 2.0  | 1.0%   |  22.4%   |  30.3   |   30.3  |
| 解码  |  H265   |  NV12   | 44.1 | 0.8%   |  19.6%   |  24.1   |   24.1  |
| 编码  |  NV12   |  JPEG   | 3.0  | 0.7%   |  22.2%   |  30.3   |   30.3  |
| 编码  |  NV12   |  H265   | 5.0  | 0.8%   |  22.2%   |  30.3   |   30.3  |

X86 (i7-8700K)：
| 类型  | 输入格式 | 输出格式 | 耗时 | CPU占用 | 内存占用 | 输入帧率 | 输出帧率 |
| ----  | ------- | ------- | ---- | ------ | -------- | ------- | ------- |
| 解码  |  JPEG   |  NV12   | 14.0  | 72.3%   |  0.4%   |  30.3   |   30.3  |
| 编码  |  NV12   |  JPEG   | 12.0  | 70.1%   |  0.3%   |  30.3   |   30.3  |
