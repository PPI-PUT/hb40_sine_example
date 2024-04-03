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

#include "sine_example/sine_example.hpp"
#include <cmath>
#include <iostream>

namespace sine_example
{

SineExample::SineExample()
{
}

float SineExample::getLeftHipPose() const
{
  return this->left_hip_pose_;
}

float SineExample::getRightHipPose() const
{
  return this->right_hip_pose_;
}


int64_t SineExample::foo(int64_t bar) const
{
  std::cout << "Hello World, " << bar << std::endl;
  return bar;
}

double SineExample::sine(double time_ms, double amplitude, double period, double offset) const
{
  return static_cast<double>(amplitude * std::sin(2 * M_PI / (period * 1000.0) * time_ms) + offset);
}

double SineExample::sine(double time) const
{
  return static_cast<double>(std::sin(time));
}

}  // namespace sine_example
