// Copyright 2021 Research Institute of Systems Planning, Inc.
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

#include "rclcpp/rclcpp.hpp"

#include "tilde/tilde_publisher.hpp"
#include "tilde_msg/msg/pub_info.hpp"

using tilde::TildePublisherBase;
using tilde::InputInfo;
using tilde_msg::msg::PubInfo;

class TestTildePublisher : public ::testing::Test
{
public:
  void SetUp()
  {
  }
};

void expect_near(
  const rclcpp::Time && lhs,
  const rclcpp::Time && rhs,
  uint64_t thres_ms = 1)
{
  uint64_t thres_ns = thres_ms * 1000 * 1000;
  EXPECT_NEAR(lhs.nanoseconds(), rhs.nanoseconds(), thres_ns);
}

TEST_F(TestTildePublisher, set_input_info) {
  auto clock = std::make_shared<rclcpp::Clock>();
  auto steady_clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

  TildePublisherBase pub(clock, steady_clock, "node_name");
  auto info = std::make_shared<tilde::InputInfo>();
  info->sub_time = rclcpp::Time(1, 0);
  info->sub_time_steady = steady_clock->now();
  info->has_header_stamp = true;
  info->header_stamp = rclcpp::Time(0, 1);
  const std::string TOPIC = "sample_topic";

  pub.set_input_info(
    TOPIC,
    info);

  tilde_msg::msg::PubInfo msg;
  pub.set_input_info(msg);

  EXPECT_EQ(msg.input_infos.size(), 1ul);
  EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
  EXPECT_EQ(msg.input_infos[0].sub_time, rclcpp::Time(1, 0));
  expect_near(
    rclcpp::Time(msg.input_infos[0].sub_time_steady, RCL_STEADY_TIME),
    steady_clock->now());
  EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[0].header_stamp, rclcpp::Time(0, 1));
}

TEST_F(TestTildePublisher, add_explicit_input_info_subtime_not_found) {
  // set input_info & explicit_input_info
  auto clock = std::make_shared<rclcpp::Clock>();
  auto steady_clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

  TildePublisherBase pub(clock, steady_clock, "node_name");
  auto now = clock->now();
  auto info_stamp = now - rclcpp::Duration(1, 0);
  auto search_stamp = now - rclcpp::Duration(2, 0);

  auto info = std::make_shared<tilde::InputInfo>();
  info->sub_time = now;
  info->sub_time_steady = steady_clock->now();
  info->has_header_stamp = true;
  info->header_stamp = info_stamp;
  const std::string TOPIC = "sample_topic";

  // they should be ignored
  pub.set_input_info(TOPIC, info);
  pub.set_input_info(TOPIC + "2", info);

  pub.add_explicit_input_info(
    TOPIC, search_stamp);

  tilde_msg::msg::PubInfo msg;
  pub.set_input_info(msg);

  EXPECT_EQ(msg.input_infos.size(), 1ul);
  EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
  EXPECT_EQ(msg.input_infos[0].sub_time, rclcpp::Time(0, 0));
  EXPECT_EQ(msg.input_infos[0].sub_time_steady, rclcpp::Time(0, 0));
  EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[0].header_stamp, search_stamp);
}

TEST_F(TestTildePublisher, set_explicit_subtime_sucess_then_purged) {
  // set input_info & explicit_input_info
  auto clock = std::make_shared<rclcpp::Clock>();
  auto steady_clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  TildePublisherBase pub(clock, steady_clock, "node_name");

  const std::string TOPIC = "sample_topic";
  auto now = clock->now();
  auto recv_base = now - rclcpp::Duration(10, 0);
  auto recv_base_steady = steady_clock->now();
  auto stamp_base = now - rclcpp::Duration(20, 0);
  pub.set_max_sub_callback_infos_sec(21);  // psudo-boudary condition

  for (int i = 0; i < 10; i++) {
    auto input_info = std::make_shared<InputInfo>();
    input_info->sub_time = recv_base + rclcpp::Duration(i, 0);
    input_info->sub_time_steady = recv_base_steady + rclcpp::Duration(i, 0);
    input_info->has_header_stamp = true;
    input_info->header_stamp = stamp_base + rclcpp::Duration(i, 0);

    pub.set_explicit_subtime(
      TOPIC, input_info);
  }

  // normal case, middle of data
  pub.add_explicit_input_info(
    TOPIC,
    stamp_base + rclcpp::Duration(5, 0));

  tilde_msg::msg::PubInfo msg;
  pub.set_input_info(msg);

  EXPECT_EQ(msg.input_infos.size(), 1ul);
  EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
  EXPECT_EQ(msg.input_infos[0].sub_time, recv_base + rclcpp::Duration(5, 0));
  EXPECT_EQ(msg.input_infos[0].sub_time_steady, recv_base_steady + rclcpp::Duration(5, 0));
  EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[0].header_stamp, stamp_base + rclcpp::Duration(5, 0));

  // boundary condition: the 1st element exists
  pub.add_explicit_input_info(
    TOPIC,
    stamp_base);

  pub.set_input_info(msg);

  EXPECT_EQ(msg.input_infos.size(), 1ul);
  EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
  EXPECT_EQ(msg.input_infos[0].sub_time, recv_base);
  EXPECT_EQ(msg.input_infos[0].sub_time_steady, recv_base_steady);
  EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[0].header_stamp, stamp_base);

  // 1st element should be deleted
  pub.set_max_sub_callback_infos_sec(20);  // psudo-boudary condition
  auto input_info = std::make_shared<InputInfo>();
  input_info->sub_time = recv_base + rclcpp::Duration(10, 0);
  input_info->sub_time_steady = recv_base_steady + rclcpp::Duration(10, 0);
  input_info->has_header_stamp = true;
  input_info->header_stamp = stamp_base + rclcpp::Duration(10, 0);
  pub.set_explicit_subtime(
    TOPIC, input_info);

  pub.add_explicit_input_info(
    TOPIC,
    stamp_base);

  pub.set_input_info(msg);

  // check explicit info is deleted
  EXPECT_EQ(msg.input_infos.size(), 1ul);
  EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
  EXPECT_EQ(msg.input_infos[0].sub_time, rclcpp::Time(0, 0));
  EXPECT_EQ(msg.input_infos[0].sub_time_steady, rclcpp::Time(0, 0));
  EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[0].header_stamp, stamp_base);
}

TEST_F(TestTildePublisher, set_multiple_topic) {
  auto clock = std::make_shared<rclcpp::Clock>();
  auto steady_clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  TildePublisherBase pub(clock, steady_clock, "node_name");
  const std::string TOPIC1 = "sample_topic1";
  const std::string TOPIC2 = "sample_topic2";

  auto now = clock->now();
  auto recv_base = now - rclcpp::Duration(10, 0);
  auto recv_base_steady = steady_clock->now();
  auto stamp_base = now - rclcpp::Duration(20, 0);
  pub.set_max_sub_callback_infos_sec(100);  // large value to prevent cleanup

  auto input_info = std::make_shared<InputInfo>();
  input_info->sub_time = recv_base;
  input_info->sub_time_steady = recv_base_steady;
  input_info->has_header_stamp = true;
  input_info->header_stamp = stamp_base;

  pub.set_explicit_subtime(
    TOPIC1, input_info);
  pub.add_explicit_input_info(
    TOPIC1,
    stamp_base);

  auto recv_base2 = recv_base + rclcpp::Duration(2, 1);
  auto recv_base2_steady = recv_base_steady + rclcpp::Duration(2, 1);
  auto stamp_base2 = stamp_base + rclcpp::Duration(1, 1);

  auto input_info2 = std::make_shared<InputInfo>();
  input_info2->sub_time = recv_base2;
  input_info2->sub_time_steady = recv_base2_steady;
  input_info2->has_header_stamp = true;
  input_info2->header_stamp = stamp_base2;

  pub.set_explicit_subtime(
    TOPIC2, input_info2);
  pub.add_explicit_input_info(
    TOPIC2,
    stamp_base2);

  tilde_msg::msg::PubInfo msg;
  pub.set_input_info(msg);

  EXPECT_EQ(msg.input_infos.size(), 2ul);
  int idx1 = 0;
  int idx2 = 1;
  if (msg.input_infos[0].topic_name == TOPIC2) {
    idx1 = 1;
    idx2 = 0;
  }

  EXPECT_EQ(msg.input_infos[idx1].topic_name, TOPIC1);
  EXPECT_EQ(msg.input_infos[idx1].sub_time, recv_base);
  EXPECT_EQ(msg.input_infos[idx1].sub_time_steady, recv_base_steady);
  EXPECT_EQ(msg.input_infos[idx1].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[idx1].header_stamp, stamp_base);

  EXPECT_EQ(msg.input_infos[idx2].topic_name, TOPIC2);
  EXPECT_EQ(msg.input_infos[idx2].sub_time, recv_base2);
  EXPECT_EQ(msg.input_infos[idx2].sub_time_steady, recv_base2_steady);
  EXPECT_EQ(msg.input_infos[idx2].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[idx2].header_stamp, stamp_base2);
}

TEST_F(TestTildePublisher, no_explcit_after_add_explicit) {
  auto clock = std::make_shared<rclcpp::Clock>();
  auto steady_clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  TildePublisherBase pub(clock, steady_clock, "node_name");
  pub.set_max_sub_callback_infos_sec(100);  // large value to prevent cleanup

  const std::string TOPIC = "sample_topic";

  auto now = clock->now();
  auto now_steady = steady_clock->now();
  auto stamp_base = now;

  // (1) use explicit API
  /// TOPIC subscription
  {
    auto input_info = std::make_shared<InputInfo>();
    input_info->sub_time = now;
    input_info->sub_time_steady = now_steady;
    input_info->has_header_stamp = true;
    input_info->header_stamp = stamp_base;

    pub.set_input_info(TOPIC, input_info);
    pub.set_explicit_subtime(
      TOPIC, input_info);
  }

  /// use explicit API
  pub.add_explicit_input_info(
    TOPIC,
    stamp_base);

  /// publish
  {
    tilde_msg::msg::PubInfo msg;
    pub.set_input_info(msg);

    /// check
    EXPECT_EQ(msg.input_infos.size(), 1ul);
    EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
    EXPECT_EQ(msg.input_infos[0].sub_time, now);
    EXPECT_EQ(msg.input_infos[0].sub_time_steady, now_steady);
    EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
    EXPECT_EQ(msg.input_infos[0].header_stamp, stamp_base);
  }

  // (test case1) pulish without subscription
  /// publish
  {
    tilde_msg::msg::PubInfo msg;
    pub.set_input_info(msg);
    /// check
    EXPECT_EQ(msg.input_infos.size(), 0ul);
  }

  // (test case2) pulish after subscription but no explicit API
  auto now2 = now + rclcpp::Duration(5, 0);
  auto now_steady2 = now_steady + rclcpp::Duration(5, 0);
  auto stamp_base2 = now2;

  /// sub
  {
    auto input_info = std::make_shared<InputInfo>();
    input_info->sub_time = now2;
    input_info->sub_time_steady = now_steady2;
    input_info->has_header_stamp = true;
    input_info->header_stamp = stamp_base2;

    pub.set_explicit_subtime(
      TOPIC, input_info);

    pub.set_input_info(TOPIC, input_info);
    pub.set_explicit_subtime(
      TOPIC, input_info);
  }

  /// publish
  {
    tilde_msg::msg::PubInfo msg;
    pub.set_input_info(msg);
    /// check
    EXPECT_EQ(msg.input_infos.size(), 0ul);
  }
}
