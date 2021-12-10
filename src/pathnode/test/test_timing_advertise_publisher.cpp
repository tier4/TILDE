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

#include "pathnode/timing_advertise_publisher.hpp"
#include "path_info_msg/msg/pub_info.hpp"

using pathnode::TimingAdvertisePublisherBase;
using pathnode::InputInfo;
using path_info_msg::msg::PubInfo;

class TestTimingAdvertisePublisher : public ::testing::Test
{
public:
  void SetUp()
  {
  }
};

TEST_F(TestTimingAdvertisePublisher, set_input_info) {
  auto clock = std::make_shared<rclcpp::Clock>();

  TimingAdvertisePublisherBase pub(clock);
  auto info = std::make_shared<pathnode::InputInfo>();
  info->sub_time = rclcpp::Time(1, 0);
  info->has_header_stamp = true;
  info->header_stamp = rclcpp::Time(0, 1);
  const std::string TOPIC = "sample_topic";

  pub.set_input_info(
    TOPIC,
    info);

  path_info_msg::msg::PubInfo msg;
  pub.set_input_info(msg);

  EXPECT_EQ(msg.input_infos.size(), 1ul);
  EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
  EXPECT_EQ(msg.input_infos[0].sub_time, rclcpp::Time(1, 0));
  EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[0].header_stamp, rclcpp::Time(0, 1));
}

TEST_F(TestTimingAdvertisePublisher, add_explicit_input_info_subtime_not_found) {
  // set input_info & explicit_input_info
  auto clock = std::make_shared<rclcpp::Clock>();

  TimingAdvertisePublisherBase pub(clock);
  auto now = clock->now();
  auto info_stamp = now - rclcpp::Duration(1, 0);
  auto search_stamp = now - rclcpp::Duration(2, 0);

  auto info = std::make_shared<pathnode::InputInfo>();
  info->sub_time = now;
  info->has_header_stamp = true;
  info->header_stamp = info_stamp;
  const std::string TOPIC = "sample_topic";

  // they should be ignored
  pub.set_input_info(TOPIC, info);
  pub.set_input_info(TOPIC + "2", info);

  pub.add_explicit_input_info(
    TOPIC, search_stamp);

  path_info_msg::msg::PubInfo msg;
  pub.set_input_info(msg);

  EXPECT_EQ(msg.input_infos.size(), 1ul);
  EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
  EXPECT_EQ(msg.input_infos[0].sub_time, rclcpp::Time(0, 0));
  EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[0].header_stamp, search_stamp);
}

TEST_F(TestTimingAdvertisePublisher, set_explicit_subtime_sucess_then_purged) {
  // set input_info & explicit_input_info
  auto clock = std::make_shared<rclcpp::Clock>();
  TimingAdvertisePublisherBase pub(clock);

  const std::string TOPIC = "sample_topic";
  auto now = clock->now();
  auto recv_base = now - rclcpp::Duration(10, 0);
  auto stamp_base = now - rclcpp::Duration(20, 0);
  pub.set_max_sub_callback_infos_sec(21);  // psudo-boudary condition

  for (int i = 0; i < 10; i++) {
    pub.set_explicit_subtime(
      TOPIC,
      stamp_base + rclcpp::Duration(i, 0),
      recv_base + rclcpp::Duration(i, 0));
  }

  // normal case, middle of data
  pub.add_explicit_input_info(
    TOPIC,
    stamp_base + rclcpp::Duration(5, 0));

  path_info_msg::msg::PubInfo msg;
  pub.set_input_info(msg);

  EXPECT_EQ(msg.input_infos.size(), 1ul);
  EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
  EXPECT_EQ(msg.input_infos[0].sub_time, recv_base + rclcpp::Duration(5, 0));
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
  EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[0].header_stamp, stamp_base);

  // 1st element should be deleted
  pub.set_max_sub_callback_infos_sec(20);  // psudo-boudary condition
  pub.set_explicit_subtime(
    TOPIC,
    stamp_base + rclcpp::Duration(10, 0),
    recv_base + rclcpp::Duration(10, 0));

  pub.add_explicit_input_info(
    TOPIC,
    stamp_base);

  pub.set_input_info(msg);

  EXPECT_EQ(msg.input_infos.size(), 1ul);
  EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
  EXPECT_EQ(msg.input_infos[0].sub_time, rclcpp::Time(0, 0));  // subtime info is deleted
  EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[0].header_stamp, stamp_base);
}

TEST_F(TestTimingAdvertisePublisher, set_multiple_topic) {
  auto clock = std::make_shared<rclcpp::Clock>();
  TimingAdvertisePublisherBase pub(clock);
  const std::string TOPIC1 = "sample_topic1";
  const std::string TOPIC2 = "sample_topic2";

  auto now = clock->now();
  auto recv_base = now - rclcpp::Duration(10, 0);
  auto stamp_base = now - rclcpp::Duration(20, 0);
  pub.set_max_sub_callback_infos_sec(100);  // large value to prevent cleanup

  pub.set_explicit_subtime(
    TOPIC1,
    stamp_base,
    recv_base);
  pub.add_explicit_input_info(
    TOPIC1,
    stamp_base);

  auto recv_base2 = stamp_base + rclcpp::Duration(2, 1);
  auto stamp_base2 = stamp_base + rclcpp::Duration(1, 1);

  pub.set_explicit_subtime(
    TOPIC2,
    stamp_base2,
    recv_base2);
  pub.add_explicit_input_info(
    TOPIC2,
    stamp_base2);

  path_info_msg::msg::PubInfo msg;
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
  EXPECT_EQ(msg.input_infos[idx1].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[idx1].header_stamp, stamp_base);

  EXPECT_EQ(msg.input_infos[idx2].topic_name, TOPIC2);
  EXPECT_EQ(msg.input_infos[idx2].sub_time, recv_base2);
  EXPECT_EQ(msg.input_infos[idx2].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[idx2].header_stamp, stamp_base2);
}
