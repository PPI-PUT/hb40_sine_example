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

#ifndef SINE_EXAMPLE__VISIBILITY_CONTROL_HPP_
#define SINE_EXAMPLE__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(SINE_EXAMPLE_BUILDING_DLL) || defined(SINE_EXAMPLE_EXPORTS)
    #define SINE_EXAMPLE_PUBLIC __declspec(dllexport)
    #define SINE_EXAMPLE_LOCAL
  #else  // defined(SINE_EXAMPLE_BUILDING_DLL) || defined(SINE_EXAMPLE_EXPORTS)
    #define SINE_EXAMPLE_PUBLIC __declspec(dllimport)
    #define SINE_EXAMPLE_LOCAL
  #endif  // defined(SINE_EXAMPLE_BUILDING_DLL) || defined(SINE_EXAMPLE_EXPORTS)
#elif defined(__linux__)
  #define SINE_EXAMPLE_PUBLIC __attribute__((visibility("default")))
  #define SINE_EXAMPLE_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define SINE_EXAMPLE_PUBLIC __attribute__((visibility("default")))
  #define SINE_EXAMPLE_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // SINE_EXAMPLE__VISIBILITY_CONTROL_HPP_
