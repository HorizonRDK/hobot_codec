English| [简体中文](./README_cn.md)

Getting Started with hobot_codec
=======

# Intro

The hobot_codec package is used to display image messages published by a ROS2 Node. It supports ROS standard formats and also supports subscribing via shared memory, publishing jpg/h264/h265 topics.

# Build

---

## Dependency

ROS packages:

- sensor_msgs
- hbm_img_msgs

The cv_bridge is an open-source ROS package that needs to be installed manually. Here is how to install it:

```cpp
# Method 1, directly install using apt, taking cv_bridge installation as an example
sudo apt-get install ros-foxy-cv-bridge -y

# Method 2, use rosdep to check and automatically install dependencies for pkg compilation
# Install ros package dependency download tool rosdep
sudo apt-get install python3-pip
sudo pip install rosdep
sudo rosdep init
rosdep update
# Execute dependency installation under the ros workspace path, specify the path where the pkg is located. By default, install dependencies for all pkgs, or specify to install dependencies for a specific pkg
rosdep install -i --from-path . --rosdistro foxy -y
```

## Development Environment

- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.04
- Compilation Toolchain: Linux GCC 9.3.0/Linaro GCC 9.3.0

## Compilation

Support compiling on X3 Ubuntu system and using docker cross-compilation on PC.

### Compilation of X3 version on X3 Ubuntu system

1. Confirm the compilation environment

- X3 Ubuntu system is installed on the board.- The current compilation terminal has set the TogetherROS environment variable: `source PATH/setup.bash`. Where PATH is the installation path of TogetherROS.
- ROS2 compilation tool colcon has been installed, installation command: `pip install -U colcon-common-extensions`

2. Compilation:

- `colcon build --packages-select hobot_codec --cmake-args -DPLATFORM_X3=ON`.

### Cross-compilation for X3 using Docker

1. Compilation Environment Confirmation

- Compile in Docker, and ensure that tros is installed in Docker. For details on Docker installation, cross-compilation instructions, tros compilation, and deployment instructions, refer to the README.md in the robot development platform robot_dev_config repository.
- The hbm_img_msgs package has been compiled (compilation method is in the Dependency section).

2. Compilation

- Compilation command:

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

- Where SYS_ROOT is the path of the cross-compilation system dependencies. For the specific address of this path, refer to the cross-compilation instructions in the "1. Compilation Environment Confirmation" section.

### Compilation of X86 Version on X86 Ubuntu System

1. Compilation Environment Confirmation

- X86 Ubuntu Version: Ubuntu 20.04
- OpenCV: 4.2.0

2. Compilation

- Compilation command:

  ```shell
  colcon build --packages-select hobot_codec  \
     --merge-install \
     --cmake-args \
     -DPLATFORM_X86=ON \
     -DTHIRD_PARTY=`pwd`/../sysroot_docker
  ```
# Usage

## X3 Ubuntu System

After successful compilation, copy the generated installation path to the Horizon X3 development board (if compiling on X3, ignore the copying step), and execute the following command to run:

```shell
export COLCON_CURRENT_PREFIX=./install
source ./install/local_setup.sh
ros2 run hobot_codec hobot_codec_republish
```

### Current Parameter List

| Parameter Name   | Meaning                      | Value                                         | Default Value         |
| ---------------- | ---------------------------- | --------------------------------------------- | --------------------- |
| channel          | Processing channel number     | 0-3                                           | 0                     |
| in_mode          | Data transmission method to access | ros/shared_mem                          | ros                   |
| out_mode         | Data transmission method to send out | ros/shared_mem                        | ros                   |
| in_format        | Subscribed data format        | bgr8/rgb8/nv12/jpeg/h264/h265     | bgr8                  |
| out_format       | Format of published data after processing | bgr8/rgb8/nv12/jpeg/h264/h265 | jpeg                  |
| sub_topic        | Topic name to subscribe to    | Any string, but must be published by another node | /image_raw        |
| pub_topic        | Topic name to publish         | Any string                                    | /image_raw/compressed |
| enc_qp           | 264/265 encoding quality      | Float number 0-100                             | 10.0                  |
| jpg_quality      | JPEG encoding quality         | Float number 0-100                             | 60.0                  |
| input_framerate  | Input frame rate, actual frame rate of input data | Positive integers               | 30                    |
| output_framerate | Output frame rate, only supported in encoding mode | Positive integers, less than or equal to input frame rate | -1 (frame rate control is not enabled) |
| dump_output      | Configuration for storing encoding/decoding output | True (store), False (do not store)  | False                 |

### Notes

1. The encoding and decoding require the image resolution's width and height to be aligned with 8 bytes.
2. The X86 version only supports mutual conversion between bgr8/rgb8/nv12 and jpeg.
3. When decoding h264 or h265, hobot_codec needs to start parsing from the first frame of the video.
4. For non-zero copy data transmission methods (when in_mode/out_mode parameters are set to ros), when `in_format/out_format` parameters are set to `jpeg`, the data type of the subscribed/published topic is `sensor_msgs::msg::CompressedImage`; when parameters are set to `bgr8/rgb8/nv12`, the data type of the subscribed/published topic is `sensor_msgs::msg::Image`.

### Run Option 1: Command Line

1. Subscribe to NV12 format images published by MIPI camera, encode them into JPEG images, and store the encoded images:

```shell
# MIPI camera publishes NV12 format images via zero-copy
ros2 run mipi_cam mipi_cam --ros-args -p out_format:=nv12 -p io_method:=shared_mem --log-level info

# Encode to JPEG
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=shared_mem -p out_format:=jpeg -p sub_topic:=/hbmem_img -p dump_output:=True
```

2. Subscribe to NV12 format images published by the image publishing tool, test encoding:

```shell
# Publish NV12 format images using the image publishing tool
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .
ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/test1.jpg -p output_image_w:=960 -p output_image_h:=544 -p image_format:=jpg -p source_image_w:=960 -p source_image_h:=544

# Encode into jpeg
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=shared_mem -p out_format:=jpeg -p sub_topic:=/hbmem_img -p dump_output:=False

# Encode into h264
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=shared_mem -p out_format:=h264 -p sub_topic:=/hbmem_img -p dump_output:=False

# Encode into h265
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=shared_mem -p out_format:=h265 -p sub_topic:=/hbmem_img -p dump_output:=False
```

2. Subscribe to H264 video and decode it into NV12 format images:

```shell
# Decode into nv12 images
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=h264 -p out_mode:=shared_mem -p out_format:=nv12 -p sub_topic:=/hbmem_img -p dump_output:=False

# Publish h264 video using the image publishing tool
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .
ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/test1.h264 -p image_format:=h264
```

3. Subscribe to H265 video and decode it into NV12 format images:

```shell
# Decode into nv12 images
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=h265 -p out_mode:=shared_mem -p out_format:=nv12 -p sub_topic:=/hbmem_img -p dump_output:=False

# Publish h265 video using the image publishing tool
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .
ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/sky.h265 -p image_format:=h265
```

4. Subscribe to JPEG images and decode them into NV12 format:

```shell
# Subscribe to jpeg images and decode into nv12 images
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=jpeg -p out_mode:=shared_mem -p out_format:=nv12 -p sub_topic:=/image_jpeg -p dump_output:=False

# Publish JPEG format images using the image publishing tool
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .
ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/test1.jpg -p output_image_w:=960 -p output_image_h:=544 -p image_format:=jpg -p is_compressed_img_pub:=True -p msg_pub_topic_name:=/image_jpeg
```

### Running method 2, using launch files

```shell
source /opt/tros/setup.bash

ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37
```

```shell
source /opt/tros/setup.bash

ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=jpeg codec_sub_topic:=/hbmem_img codec_pub_topic:=/image_jpeg
```

## X3 Linaro System

Copy the install directory compiled in docker to the Linaro system, for example: /userdata
First specify the path of the dependent libraries, for example:
`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/userdata/install/lib`

Modify the path of ROS_LOG_DIR, otherwise the logs will be created in the /home directory, need to run `mount -o remount,rw /` to create logs under /home
`export ROS_LOG_DIR=/userdata/`

Run hobot_codec_republish

```shell
// Default parameter method
/userdata/install/lib/hobot_codec/hobot_codec_republish
// Parameter passing method
#/userdata/install/lib/hobot_codec/hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=ros -p in_format:=nv12 -p out_mode:=ros -p out_format:=jpeg -p sub_topic:=/image_raw -p pub_topic:=/image_jpeg
```

## X86 Ubuntu System

After successful compilation, run the following command

```shell
source ./install/local_setup.sh

# Image publisher tool publishes images in NV12 format
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .
ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/test1.jpg -p output_image_w:=960 -p output_image_h:=544 -p image_format:=jpg -p source_image_w:=960 -p source_image_h:=544

# Encode to jpeg
ros2 run hobot_codec hobot_codec_republish --ros-args -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=shared_mem -p out_format:=jpeg -p sub_topic:=/hbmem_img -p dump_output:=False
```

## Advanced Usage ExampleSequential (the previous node hobot_codec_republish encodes the pub data of the previous codec node, and the subsequent node hobot_codec_republish sub decodes it) test for encoding and decoding:

```shell
export ROS_DOMAIN_ID=***
# Configure ROS_DOMAIN_ID to avoid interference among multiple machines. This needs to be executed in every terminal to subscribe to data.

ros2 run mipi_cam mipi_cam --ros-args --log-level info --ros-args -p out_format:=nv12 -p io_method:=shared_mem -p image_width:=640 -p image_height:=480
# Run mipi_cam to publish nv12 format images via shared memory

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=nv12 -p out_mode:=shared_mem -p out_format:=h265 -p sub_topic:=/hbmem_img -p pub_topic:=/image_h265
# Run codec to subscribe to nv12 format images, encode and publish as h265 format videos, topic: image_h265, using shared memory

ros2 run hobot_codec hobot_codec_republish --ros-args -p channel:=1 -p in_mode:=shared_mem -p in_format:=h265 -p out_mode:=shared_mem -p out_format:=nv12 -p sub_topic:=/image_h265 -p pub_topic:=/image_nv12
# Run codec to subscribe to h265 format videos (through shared memory, topic must be: image_h265), decode and publish as nv12 format images

```

## Performance Display

1. Test Methods

- Hardware: RDK X3 4G version, RDK Ultra development board

- Lock CPU frequency: `echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor`

- CPU usage is viewed by using the top command to check the single-core CPU usage of the test process.

- Time consumption indicates the time interval from subscribing to data to completing encoding/decoding and publishing data, with time statistics in milliseconds and taking the average.

- Use the image publishing tool hobot_image_publisher to publish images/videos via zero-copy for testing encoding/decoding, with a resolution of 1920x1080.

- Test commands

| Type  | Input Format | Output Format | Execution Command |
| ----  | ------------ | ------------ | ----------------- |
| Decode | JPEG        | NV12         | ros2 launch hobot_codec hobot_codec_benchmark.launch.py image_source:=1920x1080.jpeg image_format:=jpeg codec_in_format:=jpeg codec_out_format:=nv12 |
| Decode | H264        | NV12         | ros2 launch hobot_codec hobot_codec_benchmark.launch.py image_source:=1920x1080.h264 image_format:=h264 codec_in_format:=h264 codec_out_format:=nv12 |
| Decode | H265        | NV12         | ros2 launch hobot_codec hobot_codec_benchmark.launch.py image_source:=1920x1080.h265 image_format:=h265 codec_in_format:=h265 codec_out_format:=nv12 |
| Encode | NV12        | JPEG         | ros2 launch hobot_codec hobot_codec_benchmark.launch.py image_source:=1920x1080.nv12 image_format:=nv12 codec_in_format:=nv12 codec_out_format:=jpeg |
| Encode | NV12        | H264         | ros2 launch hobot_codec hobot_codec_benchmark.launch.py image_source:=1920x1080.nv12 image_format:=nv12 codec_in_format:=nv12 codec_out_format:=h264 |
| Encode | NV12        | H265         | ros2 launch hobot_codec hobot_codec_benchmark.launch.py image_source:=1920x1080.nv12 image_format:=nv12 codec_in_format:=nv12 codec_out_format:=h265 |

2. Test Results

X3 Specs:

| Type  | Input Format | Output Format | Time Consumption | CPU Usage | Memory Usage | Input FPS | Output FPS |
| ----  | ------------ | ------------  | ---------------  | --------- | ------------ | --------- | ---------- |
| Decode | JPEG        | NV12          | 4.4 ms           | 17.0%     | 0.7%        | 30.3      | 30.3       |
| Decoding | H264 | NV12 | 88.3 | 9.7% | 0.7% | 24.1 | 24.1 |
| Decoding | H265 | NV12 | 87.0 | 9.6% | 0.7% | 24.1 | 24.1 |
| Encoding | NV12 | JPEG | 4.7 | 13.0% | 0.8% | 30.3 | 30.3 |
| Encoding | NV12 | H264 | 5.5 | 11.0% | 0.8% | 30.3 | 30.3 |
| Encoding | NV12 | H265 | 5.7 | 11.0% | 0.8% | 30.3 | 30.3 |

RDK Ultra

| Type   | Input Format | Output Format | Time  | CPU Usage | Memory Usage | Input FPS | Output FPS |
| ----   | ------------ | ------------ | ----- | --------- | ------------ | --------- | ---------- |
| Decoding | JPEG       | NV12         | 2.0   | 1.0%      | 22.4%        | 30.3      | 30.3       |
| Decoding | H265       | NV12         | 44.1  | 0.8%      | 19.6%        | 24.1      | 24.1       |
| Encoding | NV12       | JPEG         | 3.0   | 0.7%      | 22.2%        | 30.3      | 30.3       |
| Encoding | NV12       | H265         | 5.0   | 0.8%      | 22.2%        | 30.3      | 30.3       |

X86 (i7-8700K):
| Type   | Input Format | Output Format | Time  | CPU Usage | Memory Usage | Input FPS | Output FPS |
| ----   | ------------ | ------------ | ----- | --------- | ------------ | --------- | ---------- |
| Decoding | JPEG       | NV12         | 14.0  | 72.3%     | 0.4%         | 30.3      | 30.3       |
| Encoding | NV12       | JPEG         | 12.0  | 70.1%     | 0.3%         | 30.3      | 30.3       |
