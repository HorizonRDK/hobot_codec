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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'codec_channel',
            default_value='1',
            description='hobot codec channel'),
        DeclareLaunchArgument(
            'codec_in_mode',
            default_value='shared_mem',
            description='image input mode'),
        DeclareLaunchArgument(
            'codec_in_format',
            default_value='nv12',
            description='image input format'),
        DeclareLaunchArgument(
            'codec_out_mode',
            default_value='ros',
            description='image output mode'),
        DeclareLaunchArgument(
            'codec_out_format',
            default_value='jpeg',
            description='image ouput format'),
        DeclareLaunchArgument(
            'codec_sub_topic',
            default_value='/hbmem_img',
            description='subscribe topic name'),
        DeclareLaunchArgument(
            'codec_pub_topic',
            default_value='/image_jpeg',
            description='publish topic name'),
        DeclareLaunchArgument(
            'codec_enc_qp',
            default_value='10.0',
            description='h264/h265 encoding quality, 0-100'),
        DeclareLaunchArgument(
            'codec_jpg_quality',
            default_value='60.0',
            description='mjpeg encoding quality, 0-100'),
        DeclareLaunchArgument(
            'codec_input_framerate',
            default_value='30',
            description='image input framerate'),
        DeclareLaunchArgument(
            'codec_output_framerate',
            default_value='-1',
            description='image output framerate'),
        DeclareLaunchArgument(
            'codec_dump_output',
            default_value='False',
            description='Dump codec output configuration'),
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            output='screen',
            parameters=[
                {"channel": LaunchConfiguration('codec_channel')},
                {"in_mode": LaunchConfiguration('codec_in_mode')},
                {"in_format": LaunchConfiguration('codec_in_format')},
                {"out_mode": LaunchConfiguration('codec_out_mode')},
                {"out_format": LaunchConfiguration('codec_out_format')},
                {"sub_topic": LaunchConfiguration('codec_sub_topic')},
                {"pub_topic": LaunchConfiguration('codec_pub_topic')},
                {"enc_qp": LaunchConfiguration('codec_enc_qp')},
                {"jpg_quality": LaunchConfiguration('codec_jpg_quality')},
                {"input_framerate": LaunchConfiguration('codec_input_framerate')},
                {"output_framerate": LaunchConfiguration('codec_output_framerate')},
                {"dump_output": LaunchConfiguration('codec_dump_output')}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        ),
    ])
