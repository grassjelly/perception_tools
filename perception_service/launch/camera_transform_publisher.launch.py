# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    publisher_config = PathJoinSubstitution(
        [FindPackageShare('perception_service'), 'config', 'camera_transform_publisher.yaml']
    )

    return LaunchDescription([
        Node(
            package='perception_service',
            executable='camera_transform_publisher',
            name='camera_transform_publisher',
            output='screen',
            parameters=[publisher_config]
        )
    ])