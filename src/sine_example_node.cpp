// Copyright 2024 Maciej Krupka
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sine_example/sine_example_node.hpp"

namespace sine_example
{

SineExampleNode::SineExampleNode(const rclcpp::NodeOptions & options)
:  Node("sine_example", options)
{
  sine_example_ = std::make_unique<sine_example::SineExample>();
  // sine_example_->foo(param_name_);

  //   rclcpp::QoS getMabQosReliable()
  rclcpp::QoS qos(1);
  qos.reliable();
  qos.durability_volatile();

  // rclcpp::QoS getMabQosRT()
  rclcpp::QoS qosRT(1);
  qos.best_effort();
  qos.durability_volatile();

  cmd_.source_node = "sine_example";
  cmd_.name = std::vector<std::string>{"fr_j0", "fr_j1", "fr_j2", "fl_j0", "fl_j1",
    "fl_j2", "rl_j0", "rl_j1", "rl_j2", "rr_j0", "rr_j1", "rr_j2", "sp_j0"};
  // fill position, velocity, effort float32 with 0.0
  cmd_.kp = std::vector<float>(cmd_.name.size(), 2.0);
  cmd_.kd = std::vector<float>(cmd_.name.size(), 0.2);
  cmd_.t_pos = std::vector<float>{sine_example_->getRightHipPose(),
    0.6000000238418579, -1.2000000476837158,
    sine_example_->getLeftHipPose(),
    -0.6000000238418579, 1.2000000476837158,
    sine_example_->getRightHipPose(),
    -0.6000000238418579, 1.2000000476837158,
    sine_example_->getLeftHipPose(),
    0.6000000238418579, -1.2000000476837158,
    0.0};
  cmd_.t_vel = std::vector<float>(cmd_.name.size(), 0.0);
  cmd_.t_trq = std::vector<float>(cmd_.name.size(), 0.0);
  steady_clock_ = std::chrono::steady_clock::now();
  timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(5),
    std::bind(&SineExampleNode::controlLoop, this));
  pub_ = this->create_publisher<JointCommand>("~/output/joint_command", qosRT);
  pub_debug_ = this->create_publisher<JointCommand>("~/output/debug/joint_command", qosRT);
  // change topic to robot state
  sub_ = this->create_subscription<BridgeData>(
    "~/input/joint_state", qos,
    std::bind(&SineExampleNode::jointStateCallback, this, std::placeholders::_1));
}

void SineExampleNode::controlLoop()
{
  cmd_.header.stamp = this->now();
  auto now = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - steady_clock_).count();
  cmd_.kp[5] = 100.0;
  cmd_.kd[5] = 0.1;
  cmd_.t_pos[5] = sine_example_->sine(static_cast<double>(elapsedTime), 0.6, 0.5, 1.2);
  // cmd_.t_pos[5] = sine_example_->sine(static_cast<double>(time_));
  pub_->publish(cmd_);
  pub_debug_->publish(cmd_);
  time_++;
  RCLCPP_INFO(get_logger(), "Publishing mili: '%f'", static_cast<double>(elapsedTime));
  RCLCPP_INFO(get_logger(), "Publishing: '%d'", time_);
}  // namespace sine_example

void SineExampleNode::jointStateCallback(const BridgeData::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "I heard: '%f'", msg->joint_position[0]);
}

}  // namespace sine_example
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sine_example::SineExampleNode)
