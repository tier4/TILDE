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
  TimingAdvertisePublisherBase pub;
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

TEST_F(TestTimingAdvertisePublisher, set_explicit_input_info) {
  // set input_info & explicit_input_info
  TimingAdvertisePublisherBase pub;
  auto info = std::make_shared<pathnode::InputInfo>();
  info->sub_time = rclcpp::Time(1, 0);
  info->has_header_stamp = true;
  info->header_stamp = rclcpp::Time(0, 1);
  const std::string TOPIC = "sample_topic";

  // they should be ignored
  pub.set_input_info(TOPIC, info);
  pub.set_input_info(TOPIC + "2", info);

  pub.set_explicit_input_info(
    TOPIC,
    rclcpp::Time(0, 2));

  path_info_msg::msg::PubInfo msg;
  pub.set_input_info(msg);

  EXPECT_EQ(msg.input_infos.size(), 1ul);
  EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
  // TODO(y-okumura-isp) implement me
  // EXPECT_EQ(msg.input_infos[0].sub_time, rclcpp::Time(0, 0));
  EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[0].header_stamp, rclcpp::Time(0, 2));
}

TEST_F(TestTimingAdvertisePublisher, set_explicit_subtime) {
  // set input_info & explicit_input_info
  TimingAdvertisePublisherBase pub;
  const std::string TOPIC = "sample_topic";

  for (int i = 0; i < 10; i++) {
    pub.set_explicit_subtime(
      TOPIC,
      rclcpp::Time(i, 0),
      rclcpp::Time(i, 1));
  }

  pub.set_explicit_input_info(
    TOPIC,
    rclcpp::Time(5, 0));

  path_info_msg::msg::PubInfo msg;
  pub.set_input_info(msg);

  EXPECT_EQ(msg.input_infos.size(), 1ul);
  EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
  EXPECT_EQ(msg.input_infos[0].sub_time, rclcpp::Time(5, 1));
  EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[0].header_stamp, rclcpp::Time(5, 0));

  // the 1st element exists
  pub.set_explicit_input_info(
    TOPIC,
    rclcpp::Time(0, 0));

  pub.set_input_info(msg);

  EXPECT_EQ(msg.input_infos.size(), 1ul);
  EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
  EXPECT_EQ(msg.input_infos[0].sub_time, rclcpp::Time(0, 1));
  EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[0].header_stamp, rclcpp::Time(0, 0));


  // 1st element should be deleted
  pub.set_explicit_subtime(
    TOPIC,
    rclcpp::Time(10, 0),
    rclcpp::Time(10, 1));

  pub.set_explicit_input_info(
    TOPIC,
    rclcpp::Time(0, 0));

  pub.set_input_info(msg);

  EXPECT_EQ(msg.input_infos.size(), 1ul);
  EXPECT_EQ(msg.input_infos[0].topic_name, TOPIC);
  EXPECT_EQ(msg.input_infos[0].sub_time, rclcpp::Time(0, 0));  // Time(0, 0) info is deleted
  EXPECT_EQ(msg.input_infos[0].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[0].header_stamp, rclcpp::Time(0, 0));
}

TEST_F(TestTimingAdvertisePublisher, set_multiple_topic) {
  TimingAdvertisePublisherBase pub;
  const std::string TOPIC1 = "sample_topic1";
  const std::string TOPIC2 = "sample_topic2";

  pub.set_explicit_subtime(
    TOPIC1,
    rclcpp::Time(1, 0),
    rclcpp::Time(2, 0));
  pub.set_explicit_input_info(
    TOPIC1,
    rclcpp::Time(1, 0));

  pub.set_explicit_subtime(
    TOPIC2,
    rclcpp::Time(1, 1),
    rclcpp::Time(2, 1));
  pub.set_explicit_input_info(
    TOPIC2,
    rclcpp::Time(1, 1));

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
  EXPECT_EQ(msg.input_infos[idx1].sub_time, rclcpp::Time(2, 0));
  EXPECT_EQ(msg.input_infos[idx1].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[idx1].header_stamp, rclcpp::Time(1, 0));

  EXPECT_EQ(msg.input_infos[idx2].topic_name, TOPIC2);
  EXPECT_EQ(msg.input_infos[idx2].sub_time, rclcpp::Time(2, 1));
  EXPECT_EQ(msg.input_infos[idx2].has_header_stamp, true);
  EXPECT_EQ(msg.input_infos[idx2].header_stamp, rclcpp::Time(1, 1));
}
