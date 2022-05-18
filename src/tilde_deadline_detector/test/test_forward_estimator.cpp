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
using tilde_msg::msg::SubTopicTimeInfo;
using TimeMsg = builtin_interfaces::msg::Time;

TimeMsg get_time(int sec, int nsec)
{
  TimeMsg t;
  t.sec = sec;
  t.nanosec = nsec;
  return t;
}

TEST(ForwardEstimator, one_sensor)
{
  auto fe = ForwardEstimator();
  const std::string topic = "sensor";

  const auto time_msg1 = get_time(10, 100);
  rclcpp::Time time1(time_msg1);

  auto is0 = fe.get_input_sources(topic, time1);
  EXPECT_EQ(is0.size(), 0u);

  // msg 1
  auto pub_info1 = std::make_shared<PubInfo>();
  pub_info1->output_info.topic_name = topic;
  pub_info1->output_info.has_header_stamp = true;
  pub_info1->output_info.header_stamp = time_msg1;

  fe.add(pub_info1);
  auto is1 = fe.get_input_sources(topic, time1);
  EXPECT_NE(is1.find(topic), is1.end());
  EXPECT_EQ(is1[topic].size(), 1u);
  EXPECT_EQ(*is1[topic].begin(), time1);

  // msg 2
  const auto time_msg2 = get_time(11, 110);
  rclcpp::Time time2(time_msg2);
  auto pub_info2 = std::make_shared<PubInfo>();
  pub_info2->output_info.topic_name = topic;
  pub_info2->output_info.has_header_stamp = true;
  pub_info2->output_info.header_stamp = time_msg2;

  fe.add(pub_info2);
  auto is2 = fe.get_input_sources(topic, time2);
  EXPECT_NE(is2.find(topic), is2.end());
  EXPECT_EQ(is2[topic].size(), 1u);
  EXPECT_EQ(*is2[topic].begin(), time2);
}

TEST(ForwardEstimator, two_sensors)
{
  auto fe = ForwardEstimator();
  const std::string topic1 = "sensor1";
  const std::string topic2 = "sensor2";

  const auto time_msg1 = get_time(10, 100);
  rclcpp::Time time1(time_msg1);

  // sensor1 msg1
  const auto time_msg11 = get_time(11, 110);
  rclcpp::Time time11(time_msg11);
  auto pub_info11 = std::make_shared<PubInfo>();
  pub_info11->output_info.topic_name = topic1;
  pub_info11->output_info.has_header_stamp = true;
  pub_info11->output_info.header_stamp = time_msg11;

  fe.add(pub_info11);

  auto is11 = fe.get_input_sources(topic1, time11);
  EXPECT_NE(is11.find(topic1), is11.end());
  EXPECT_EQ(is11[topic1].size(), 1u);
  EXPECT_EQ(*is11[topic1].begin(), time11);

  // sensor2 msg1
  const auto time_msg21 = get_time(21, 210);
  rclcpp::Time time21(time_msg21);
  auto pub_info21 = std::make_shared<PubInfo>();
  pub_info21->output_info.topic_name = topic2;
  pub_info21->output_info.has_header_stamp = true;
  pub_info21->output_info.header_stamp = time_msg21;

  fe.add(pub_info21);

  auto is21 = fe.get_input_sources(topic2, time21);
  EXPECT_NE(is21.find(topic2), is21.end());
  EXPECT_EQ(is21[topic2].size(), 1u);
  EXPECT_EQ(*is21[topic2].begin(), time21);

  // skew
  auto is1_21 = fe.get_input_sources(topic1, time21); // topic1 of time21
  EXPECT_EQ(is1_21.find(topic1), is1_21.end());
}

TEST(ForwardEstimator, simple_flow_stamp_preserved)
{
  // A -> B -> C. header.stamp is preserved
  auto fe = ForwardEstimator();
  const std::string topic1 = "topicA";
  const std::string topic2 = "topicB";
  const std::string topic3 = "topicC";

  // sensor1 msg1
  const auto time_msg11 = get_time(11, 110);
  rclcpp::Time time11(time_msg11);
  auto pub_info11 = std::make_shared<PubInfo>();
  pub_info11->output_info.topic_name = topic1;
  pub_info11->output_info.has_header_stamp = true;
  pub_info11->output_info.header_stamp = time_msg11;
  fe.add(pub_info11);

  // sensor2 msg1
  rclcpp::Time time21 = time11;
  auto pub_info21 = std::make_shared<PubInfo>();
  pub_info21->output_info.topic_name = topic2;
  pub_info21->output_info.has_header_stamp = true;
  pub_info21->output_info.header_stamp = time21;
  auto ii21 = SubTopicTimeInfo();
  ii21.topic_name = topic1;
  ii21.has_header_stamp = true;
  ii21.header_stamp = time11;
  pub_info21->input_infos.push_back(ii21);
  fe.add(pub_info21);

  // sensor3 msg1
  rclcpp::Time time31 = time11;
  auto pub_info31 = std::make_shared<PubInfo>();
  pub_info31->output_info.topic_name = topic3;
  pub_info31->output_info.has_header_stamp = true;
  pub_info31->output_info.header_stamp = time31;
  auto ii31 = SubTopicTimeInfo();
  ii31.topic_name = topic2;
  ii31.has_header_stamp = true;
  ii31.header_stamp = time21;
  pub_info31->input_infos.push_back(ii31);
  fe.add(pub_info31);

  // check
  auto is21 = fe.get_input_sources(topic2, time21);
  EXPECT_EQ(is21.size(), 1u);
  EXPECT_NE(is21.find(topic1), is21.end());
  EXPECT_EQ(is21[topic1].size(), 1u);
  EXPECT_EQ(*is21[topic1].begin(), time11);

  auto is31 = fe.get_input_sources(topic3, time31);
  EXPECT_EQ(is31.size(), 1u);
  EXPECT_NE(is31.find(topic1), is31.end());
  EXPECT_EQ(is31[topic1].size(), 1u);
  EXPECT_EQ(*is31[topic1].begin(), time11);
}

TEST(ForwardEstimator, simple_flow_stamp_update)
{
  // A -> B -> C. header.stamp is updated
  auto fe = ForwardEstimator();
  const std::string topic1 = "topicA";
  const std::string topic2 = "topicB";
  const std::string topic3 = "topicC";

  // sensor1 msg1
  const auto time_msg11 = get_time(11, 110);
  rclcpp::Time time11(time_msg11);
  auto pub_info11 = std::make_shared<PubInfo>();
  pub_info11->output_info.topic_name = topic1;
  pub_info11->output_info.has_header_stamp = true;
  pub_info11->output_info.header_stamp = time_msg11;
  fe.add(pub_info11);

  // sensor2 msg1
  const auto time_msg21 = get_time(21, 210);
  rclcpp::Time time21(time_msg21);
  auto pub_info21 = std::make_shared<PubInfo>();
  pub_info21->output_info.topic_name = topic2;
  pub_info21->output_info.has_header_stamp = true;
  pub_info21->output_info.header_stamp = time21;
  auto ii21 = SubTopicTimeInfo();
  ii21.topic_name = topic1;
  ii21.has_header_stamp = true;
  ii21.header_stamp = time11;
  pub_info21->input_infos.push_back(ii21);
  fe.add(pub_info21);

  // sensor3 msg1
  const auto time_msg31 = get_time(31, 310);
  rclcpp::Time time31(time_msg31);
  auto pub_info31 = std::make_shared<PubInfo>();
  pub_info31->output_info.topic_name = topic3;
  pub_info31->output_info.has_header_stamp = true;
  pub_info31->output_info.header_stamp = time31;
  auto ii31 = SubTopicTimeInfo();
  ii31.topic_name = topic2;
  ii31.has_header_stamp = true;
  ii31.header_stamp = time21;
  pub_info31->input_infos.push_back(ii31);
  fe.add(pub_info31);

  // check
  auto is21 = fe.get_input_sources(topic2, time21);
  EXPECT_EQ(is21.size(), 1u);
  EXPECT_NE(is21.find(topic1), is21.end());
  EXPECT_EQ(is21[topic1].size(), 1u);
  EXPECT_EQ(*is21[topic1].begin(), time11);

  auto is31 = fe.get_input_sources(topic3, time31);
  EXPECT_EQ(is31.size(), 1u);
  EXPECT_NE(is31.find(topic1), is31.end());
  EXPECT_EQ(is31[topic1].size(), 1u);
  EXPECT_EQ(*is31[topic1].begin(), time11);
}

TEST(ForwardEstimator, merged_flow)
{
  // A,B -> C. header.stamp
  auto fe = ForwardEstimator();
  const std::string topic1 = "topicA";
  const std::string topic2 = "topicB";
  const std::string topic3 = "topicC";

  // sensor1 msg1
  const auto time_msg11 = get_time(11, 110);
  rclcpp::Time time11(time_msg11);
  auto pub_info11 = std::make_shared<PubInfo>();
  pub_info11->output_info.topic_name = topic1;
  pub_info11->output_info.has_header_stamp = true;
  pub_info11->output_info.header_stamp = time_msg11;
  fe.add(pub_info11);

  // sensor2 msg1
  const auto time_msg21 = get_time(21, 210);
  rclcpp::Time time21(time_msg21);
  auto pub_info21 = std::make_shared<PubInfo>();
  pub_info21->output_info.topic_name = topic2;
  pub_info21->output_info.has_header_stamp = true;
  pub_info21->output_info.header_stamp = time21;
  fe.add(pub_info21);

  // sensor2 msg2
  const auto time_msg22 = get_time(22, 220);
  rclcpp::Time time22(time_msg22);
  auto pub_info22 = std::make_shared<PubInfo>();
  pub_info22->output_info.topic_name = topic2;
  pub_info22->output_info.has_header_stamp = true;
  pub_info22->output_info.header_stamp = time22;
  fe.add(pub_info22);

  // sensor3 msg1 consists of msg 11, 21, 22
  const auto time_msg31 = get_time(31, 310);
  rclcpp::Time time31(time_msg31);
  auto pub_info31 = std::make_shared<PubInfo>();
  pub_info31->output_info.topic_name = topic3;
  pub_info31->output_info.has_header_stamp = true;
  pub_info31->output_info.header_stamp = time31;

  auto ii11 = SubTopicTimeInfo();
  ii11.topic_name = topic1;
  ii11.has_header_stamp = true;
  ii11.header_stamp = time11;
  pub_info31->input_infos.push_back(ii11);

  auto ii21 = SubTopicTimeInfo();
  ii21.topic_name = topic2;
  ii21.has_header_stamp = true;
  ii21.header_stamp = time21;
  pub_info31->input_infos.push_back(ii21);

  auto ii22 = SubTopicTimeInfo();
  ii22.topic_name = topic2;
  ii22.has_header_stamp = true;
  ii22.header_stamp = time22;
  pub_info31->input_infos.push_back(ii22);

  fe.add(pub_info31);

  // check
  auto is31 = fe.get_input_sources(topic3, time31);
  EXPECT_EQ(is31.size(), 2u);
  EXPECT_NE(is31.find(topic1), is31.end());
  EXPECT_EQ(is31[topic1].size(), 1u);
  EXPECT_EQ(*is31[topic1].begin(), time11);

  EXPECT_NE(is31.find(topic2), is31.end());
  EXPECT_EQ(is31[topic2].size(), 2u);
  EXPECT_NE(is31[topic2].find(time21), is31[topic2].end());
  EXPECT_NE(is31[topic2].find(time22), is31[topic2].end());
}
