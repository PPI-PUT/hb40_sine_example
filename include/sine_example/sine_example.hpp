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

#ifndef SINE_EXAMPLE__SINE_EXAMPLE_HPP_
#define SINE_EXAMPLE__SINE_EXAMPLE_HPP_

#include <cstdint>

#include "sine_example/visibility_control.hpp"


namespace sine_example
{

class SINE_EXAMPLE_PUBLIC SineExample
{
public:
  SineExample();
  float getRightHipPose() const;
  float getLeftHipPose() const;
  int64_t foo(int64_t bar) const;
  double sine(double time, double amplitude, double period, double offset) const;
  double sine(double time) const;
private:
  float right_hip_pose_ = 0.1f;
  float left_hip_pose_ = -0.1f;
};

}  // namespace sine_example

#endif  // SINE_EXAMPLE__SINE_EXAMPLE_HPP_
