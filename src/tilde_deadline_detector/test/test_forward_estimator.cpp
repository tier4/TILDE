// Copyright 2022 Research Institute of Systems Planning, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "tilde_msg/msg/pub_info.hpp"
#include "tilde_msg/msg/sub_topic_time_info.hpp"
#include "tilde_deadline_detector/forward_estimator.hpp"

using tilde_deadline_detector::ForwardEstimator;

TEST(ForwardEstimator, add_sensor)
{
  SUCCEED();
}
