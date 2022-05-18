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
#include "builtin_interfaces/msg/time.hpp"
#include "tilde_msg/msg/pub_info.hpp"
#include "tilde_msg/msg/sub_topic_time_info.hpp"


#include "tilde_deadline_detector/forward_estimator.hpp"

using tilde_deadline_detector::ForwardEstimator;
using tilde_msg::msg::PubInfo;
using TimeMsg = builtin_interfaces::msg::Time;

TimeMsg get_time(int sec, int nsec)
{
  TimeMsg t;
  t.sec = sec;
  t.nanosec = nsec;
  return t;
}

TEST(ForwardEstimator, add_sensor)
{
  auto fe = ForwardEstimator();
  auto pub_info = std::make_shared<PubInfo>();
  const std::string topic = "sensor";
  const auto time_msg = get_time(10, 100);
  rclcpp::Time time(time_msg);

  auto is1 = fe.get_input_sources(topic, time);
  EXPECT_EQ(is1.size(), 0u);

  pub_info->output_info.topic_name = "sensor";
  pub_info->output_info.has_header_stamp = true;
  pub_info->output_info.header_stamp = time_msg;

  fe.add(pub_info);
  auto is2 = fe.get_input_sources(topic, time);
  EXPECT_NE(is2.find(topic), is2.end());
  EXPECT_EQ(is2[topic].size(), 1u);
  EXPECT_EQ(*is2[topic].begin(), time);
}
