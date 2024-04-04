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
  using std::placeholders::_1;
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
  // cmd_.kp = std::vector<float>(cmd_.name.size(), 30.0);
  kd_j0_ = this->declare_parameter("kd_j0", kd_j0_);
  kd_j1_ = this->declare_parameter("kd_j1", kd_j1_);
  kd_j2_ = this->declare_parameter("kd_j2", kd_j2_);
  kd_spine_ = this->declare_parameter("kd_spine", kd_spine_);
  kp_j0_ = this->declare_parameter("kp_j0", kp_j0_);
  kp_j1_ = this->declare_parameter("kp_j1", kp_j1_);
  kp_j2_ = this->declare_parameter("kp_j2", kp_j2_);
  kp_spine_ = this->declare_parameter("kp_spine", kp_spine_);
  amplitude_ = this->declare_parameter("amplitude", amplitude_);
  period_ = this->declare_parameter("period", period_);
  set_param_res_ =
    this->add_on_set_parameters_callback(
    std::bind(
      &SineExampleNode::onSetParam, this,
      _1));
  cmd_.kp = {
    kp_j0_, kp_j1_, kp_j2_,
    kp_j0_, kp_j1_, kp_j2_,
    kp_j0_, kp_j1_, kp_j2_,
    kp_j0_, kp_j1_, kp_j2_,
    kp_spine_
  };
  cmd_.kd = {
    kd_j0_, kd_j1_, kd_j2_,
    kd_j0_, kd_j1_, kd_j2_,
    kd_j0_, kd_j1_, kd_j2_,
    kd_j0_, kd_j1_, kd_j2_,
    kd_spine_
  };
  cmd_.t_pos = std::vector<float>{
    -0.1f, 1.0f, -1.5f,
    0.1f, -0.8f, 1.5f,
    -0.1f, -1.0f, 1.5f,
    0.1f, 0.8f, -1.5f,
    0.0f};
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
    std::bind(&SineExampleNode::jointStateCallback, this, _1));
}

void SineExampleNode::controlLoop()
{
  cmd_.header.stamp = this->now();
  auto now = std::chrono::steady_clock::now();
  auto elapsedTime =
    std::chrono::duration_cast<std::chrono::milliseconds>(now - steady_clock_).count();
  cmd_.t_pos[12] = sine_example_->sine(static_cast<double>(elapsedTime), amplitude_, period_, 0);
  // cmd_.t_pos[5] = sine_example_->sine(static_cast<double>(time_));
  pub_->publish(cmd_);
  pub_debug_->publish(cmd_);
  time_++;
  // RCLCPP_INFO(get_logger(), "Publishing mili: '%f'", static_cast<double>(elapsedTime));
  // RCLCPP_INFO(get_logger(), "Publishing: '%d'", time_);
}  // namespace sine_example

void SineExampleNode::jointStateCallback(const BridgeData::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "I heard: '%f'", msg->joint_position[0]);
}


rcl_interfaces::msg::SetParametersResult SineExampleNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  auto check_gain = [](const double & param)->bool
    {
      return param > 0.0;
    };
  result.successful = false;
  result.reason = "Failed to set parameters";
  try {
    {
      for (const auto & param : params) {
        if (param.get_name() == "kp_j0") {
          if (check_gain(param.as_double())) {
            kp_j0_ = static_cast<float>(param.as_double());
            result.successful = true;
            result.reason = "Successfully set kp_j0";
          }
        } else if (param.get_name() == "kp_j1") {
          if (check_gain(param.as_double())) {
            kp_j1_ = static_cast<float>(param.as_double());
            result.successful = true;
            result.reason = "Successfully set kp_j1";
          }
        } else if (param.get_name() == "kp_j2") {
          if (check_gain(param.as_double())) {
            kp_j2_ = static_cast<float>(param.as_double());
            result.successful = true;
            result.reason = "Successfully set kp_j2";
          }
        } else if (param.get_name() == "kd_j0") {
          if (check_gain(param.as_double())) {
            kd_j0_ = static_cast<float>(param.as_double());
            result.successful = true;
            result.reason = "Successfully set kd_j0";
          }
        } else if (param.get_name() == "kd_j1") {
          if (check_gain(param.as_double())) {
            kd_j1_ = static_cast<float>(param.as_double());
            result.successful = true;
            result.reason = "Successfully set kd_j1";
          }
        } else if (param.get_name() == "kd_j2") {
          if (check_gain(param.as_double())) {
            kd_j2_ = static_cast<float>(param.as_double());
            result.successful = true;
            result.reason = "Successfully set kd_j2";
          }
        } else if (param.get_name() == "kp_spine") {
          if (check_gain(param.as_double())) {
            kp_spine_ = static_cast<float>(param.as_double());
            result.successful = true;
            result.reason = "Successfully set kp_spine";
          }
        } else if (param.get_name() == "kd_spine") {
          if (check_gain(param.as_double())) {
            kd_spine_ = static_cast<float>(param.as_double());
            result.successful = true;
            result.reason = "Successfully set kd_spine";
          }
        } else if (param.get_name() == "amplitude") {
          if (check_gain(param.as_double())) {
            amplitude_ = static_cast<float>(param.as_double());
            result.successful = true;
            result.reason = "Successfully set amplitude";
          }
        } else if (param.get_name() == "period") {
          if (check_gain(param.as_double())) {
            period_ = static_cast<float>(param.as_double());
            result.successful = true;
            result.reason = "Successfully set period";
          }
        }
      }
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }
  if (result.successful == true) {
    RCLCPP_INFO(get_logger(), "Successfully set gain parameters");
    cmd_.kp = {
      kp_j0_, kp_j1_, kp_j2_,
      kp_j0_, kp_j1_, kp_j2_,
      kp_j0_, kp_j1_, kp_j2_,
      kp_j0_, kp_j1_, kp_j2_,
      kp_spine_
    };
    cmd_.kd = {
      kd_j0_, kd_j1_, kd_j2_,
      kd_j0_, kd_j1_, kd_j2_,
      kd_j0_, kd_j1_, kd_j2_,
      kd_j0_, kd_j1_, kd_j2_,
      kd_spine_
    };
  }
  return result;
}

}  // namespace sine_example
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sine_example::SineExampleNode)
