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

from argparse import Action
import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():
    benchmark_img_path = os.path.join(
        get_package_prefix('hobot_codec'),
        'lib/hobot_codec/config')

    print("hobot_codec benchmark_img_path is ", benchmark_img_path)

    image_source_launch_arg = DeclareLaunchArgument(
        "image_source", default_value=TextSubstitution(text="null")
    )
    image_format_launch_arg = DeclareLaunchArgument(
        "image_format", default_value=TextSubstitution(text="null")
    )
    codec_in_format_launch_arg = DeclareLaunchArgument(
        "codec_in_format", default_value=TextSubstitution(text="null")
    )
    codec_out_format_launch_arg = DeclareLaunchArgument(
        "codec_out_format", default_value=TextSubstitution(text="null")
    )

    return LaunchDescription([
        image_source_launch_arg,
        image_format_launch_arg,
        codec_in_format_launch_arg,
        codec_out_format_launch_arg,
        # 启动零拷贝环境配置node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('hobot_shm'),
                    'launch/hobot_shm.launch.py'))
        ),
        Node(
            package='hobot_image_publisher',
            executable='hobot_image_pub',
            output='screen',
            parameters=[
                {"image_source": [benchmark_img_path, "/",
                                  LaunchConfiguration('image_source')]},
                {"image_format": LaunchConfiguration('image_format')},
                {"msg_pub_topic_name": "/hbmem_img"},
                {"output_image_w": 0},
                {"output_image_h": 0},
                {"source_image_w": 1920},
                {"source_image_h": 1080},
                {"fps": 30},
                {"is_loop": True},
                {"is_shared_mem": True},
                {"is_compressed_img_pub": True}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        ),
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            output='screen',
            parameters=[
                {"in_mode": "shared_mem"},
                {"in_format": LaunchConfiguration('codec_in_format')},
                {"out_mode": "ros"},
                {"out_format": LaunchConfiguration('codec_out_format')},
                {"sub_topic": "/hbmem_img"},
                {"pub_topic": "/image_jpeg"},
                {"dump_output": False}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        ),
    ])
