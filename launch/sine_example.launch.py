# Copyright 2024 Maciej Krupka
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
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    param_path = LaunchConfiguration('sine_example_param_file').perform(context)
    if not param_path:
        param_path = PathJoinSubstitution(
            [FindPackageShare('sine_example'), 'config', 'sine_example.param.yaml']
        ).perform(context)

    sine_example_node = Node(
        package='sine_example',
        executable='sine_example_node_exe',
        name='sine_example_node',
        parameters=[
            param_path
        ],
        remappings=[
            ('~/input/joint_states', LaunchConfiguration('input_joint_state_topic')),
            ('~/output/joint_command', LaunchConfiguration('output_joint_state_topic'))
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
    )

    return [
        sine_example_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('sine_example_param_file', '')
    add_launch_arg('input_joint_state_topic', '/hb40/joint_states')
    add_launch_arg('output_joint_state_topic', '/hb40/joint_command')
    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
