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

#include "gtest/gtest.h"
#include "sine_example/sine_example.hpp"

TEST(TestSineExample, TestHello) {
  std::unique_ptr<sine_example::SineExample> sine_example_ =
    std::make_unique<sine_example::SineExample>();
  auto result = sine_example_->foo(999);
  EXPECT_EQ(result, 999);
}
