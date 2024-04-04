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

#ifndef SINE_EXAMPLE__SINE_EXAMPLE_NODE_HPP_
#define SINE_EXAMPLE__SINE_EXAMPLE_NODE_HPP_

#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "hb40_commons/msg/bridge_data.hpp"
#include "sine_example/sine_example.hpp"
#include "hb40_commons/msg/joint_command.hpp"
namespace sine_example
{
using SineExamplePtr = std::unique_ptr<sine_example::SineExample>;
using BridgeData = hb40_commons::msg::BridgeData;
using JointCommand = hb40_commons::msg::JointCommand;
class SINE_EXAMPLE_PUBLIC SineExampleNode : public rclcpp::Node
{
public:
  explicit SineExampleNode(const rclcpp::NodeOptions & options);

private:
  int time_{0};
  // kp 
  float kp_j0_{30.0f};
  float kp_j1_{30.0f};
  float kp_j2_{30.0f};
  float kp_spine_{30.0f};
  // kd
  float kd_j0_{5.0f};
  float kd_j1_{5.0f};
  float kd_j2_{5.0f};
  float kd_spine_{5.0f};
  double period_{0.5};
  double amplitude_{0.1};
  std::chrono::_V2::steady_clock::time_point steady_clock_;
  SineExamplePtr sine_example_{nullptr};
  JointCommand cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<JointCommand>::SharedPtr pub_;
  rclcpp::Publisher<JointCommand>::SharedPtr pub_debug_;
  rclcpp::Subscription<BridgeData>::SharedPtr sub_;
  void controlLoop();
  void jointStateCallback(const BridgeData::SharedPtr msg);
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);
};
}  // namespace sine_example

#endif  // SINE_EXAMPLE__SINE_EXAMPLE_NODE_HPP_
