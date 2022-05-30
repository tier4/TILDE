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

std::unique_ptr<PubInfo>
create_pubinfo(
    const std::string & topic,
    const TimeMsg & time)
{
  auto pub_info = std::make_unique<PubInfo>();
  pub_info->output_info.topic_name = topic;
  pub_info->output_info.has_header_stamp = true;
  pub_info->output_info.header_stamp = time;

  return pub_info;
}

void add_input_info(
    PubInfo * target_pub_info,
    const PubInfo * const in_pub_info)
{
  auto ii = SubTopicTimeInfo();
  ii.topic_name = in_pub_info->output_info.topic_name;
  ii.has_header_stamp = true;
  ii.header_stamp = in_pub_info->output_info.header_stamp;
  target_pub_info->input_infos.push_back(ii);
}

TEST(TestForwardEstimator, one_sensor)
{
  auto fe = ForwardEstimator();
  const std::string topic = "sensor";

  const auto time1 = get_time(10, 100);

  auto is0 = fe.get_input_sources(topic, time1);
  EXPECT_EQ(is0.size(), 0u);

  // msg 1
  auto pub_info1 = create_pubinfo(topic, time1);
  fe.add(std::move(pub_info1));

  auto is1 = fe.get_input_sources(topic, time1);
  EXPECT_NE(is1.find(topic), is1.end());
  EXPECT_EQ(is1[topic].size(), 1u);
  EXPECT_EQ(*is1[topic].begin(), time1);

  // msg 2
  auto time2 = get_time(11, 110);
  auto pub_info2 = create_pubinfo(topic, time2);

  fe.add(std::move(pub_info2));

  auto is2 = fe.get_input_sources(topic, time2);
  EXPECT_NE(is2.find(topic), is2.end());
  EXPECT_EQ(is2[topic].size(), 1u);
  EXPECT_EQ(*is2[topic].begin(), time2);
}

TEST(TestForwardEstimator, two_sensors)
{
  auto fe = ForwardEstimator();

  // sensor1 msg1
  const std::string topic1 = "sensor1";
  const auto time11 = get_time(10, 100);
  auto pub_info11 = create_pubinfo(topic1, time11);

  fe.add(std::move(pub_info11));

  auto is11 = fe.get_input_sources(topic1, time11);
  EXPECT_NE(is11.find(topic1), is11.end());
  EXPECT_EQ(is11[topic1].size(), 1u);
  EXPECT_EQ(*is11[topic1].begin(), time11);

  // sensor2 msg1
  const std::string topic2 = "sensor2";
  const auto time21 = get_time(21, 210);
  auto pub_info21 = create_pubinfo(topic2, time21);

  fe.add(std::move(pub_info21));

  auto is21 = fe.get_input_sources(topic2, time21);
  EXPECT_NE(is21.find(topic2), is21.end());
  EXPECT_EQ(is21[topic2].size(), 1u);
  EXPECT_EQ(*is21[topic2].begin(), time21);

  // skew
  auto is1_21 = fe.get_input_sources(topic1, time21);  // topic1 of time21
  EXPECT_EQ(is1_21.find(topic1), is1_21.end());
}

TEST(TestForwardEstimator, simple_flow_stamp_preserved)
{
  // A -> B -> C. header.stamp is preserved
  auto fe = ForwardEstimator();
  const std::string topic1 = "topicA";
  const std::string topic2 = "topicB";
  const std::string topic3 = "topicC";

  // sensor1 msg1
  const auto time11 = get_time(11, 110);
  auto pub_info11 = create_pubinfo(topic1, time11);

  // sensor2 msg1
  const auto time21 = time11;
  auto pub_info21 = create_pubinfo(topic2, time21);
  add_input_info(pub_info21.get(), pub_info11.get());

  // sensor3 msg1
  rclcpp::Time time31 = time11;
  auto pub_info31 = create_pubinfo(topic3, time31);
  add_input_info(pub_info31.get(), pub_info21.get());

  fe.add(std::move(pub_info11));
  fe.add(std::move(pub_info21));
  fe.add(std::move(pub_info31));

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

TEST(TestForwardEstimator, simple_flow_stamp_update)
{
  // A -> B -> C. header.stamp is updated
  auto fe = ForwardEstimator();
  const std::string topic1 = "topicA";
  const std::string topic2 = "topicB";
  const std::string topic3 = "topicC";

  // sensor1 msg1
  const auto time11 = get_time(11, 110);
  auto pub_info11 = create_pubinfo(topic1, time11);

  // sensor2 msg1
  const auto time21 = get_time(21, 210);
  auto pub_info21 = create_pubinfo(topic2, time21);
  add_input_info(pub_info21.get(), pub_info11.get());

  // sensor3 msg1
  const auto time31 = get_time(31, 310);
  auto pub_info31 = create_pubinfo(topic3, time31);
  add_input_info(pub_info31.get(), pub_info21.get());

  fe.add(std::move(pub_info11));
  fe.add(std::move(pub_info21));
  fe.add(std::move(pub_info31));

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

TEST(TestForwardEstimator, merged_flow)
{
  // A,B -> C. header.stamp
  auto fe = ForwardEstimator();
  const std::string topic1 = "topicA";
  const std::string topic2 = "topicB";
  const std::string topic3 = "topicC";

  // sensor1 msg1
  const auto time11 = get_time(11, 110);
  auto pub_info11 = create_pubinfo(topic1, time11);

  // sensor2 msg1
  const auto time21 = get_time(21, 210);
  auto pub_info21 = create_pubinfo(topic2, time21);

  // sensor2 msg2
  const auto time22 = get_time(22, 220);
  auto pub_info22 = create_pubinfo(topic2, time22);

  // sensor3 msg1 consists of msg 11, 21, 22
  const auto time31 = get_time(31, 310);
  auto pub_info31 = create_pubinfo(topic3, time31);
  add_input_info(pub_info31.get(), pub_info11.get());
  add_input_info(pub_info31.get(), pub_info21.get());
  add_input_info(pub_info31.get(), pub_info22.get());

  fe.add(std::move(pub_info11));
  fe.add(std::move(pub_info21));
  fe.add(std::move(pub_info22));
  fe.add(std::move(pub_info31));

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

TEST(TestForwardEstimator, reverse_order2)
{
  // DAG is "A -> B -> C",
  // but PubInfo comes C -> A -> B
  auto fe = ForwardEstimator();
  const std::string topic1 = "topicA";
  const std::string topic2 = "topicB";
  const std::string topic3 = "topicC";

  // PubInfo of A
  const auto time11 = get_time(11, 110);
  auto pub_info11 = create_pubinfo(topic1, time11);

  // PubInfo of B
  const auto time21 = get_time(21, 210);
  auto pub_info21 = create_pubinfo(topic2, time21);
  add_input_info(pub_info21.get(), pub_info11.get());

  // PubInfo of C
  const auto time31 = get_time(31, 310);
  auto pub_info31 = create_pubinfo(topic3, time31);
  add_input_info(pub_info31.get(), pub_info21.get());

  // add PubInfo in C -> A -> B order
  fe.add(std::move(pub_info31));
  fe.add(std::move(pub_info11));
  fe.add(std::move(pub_info21));

  auto is31 = fe.get_input_sources(topic3, time31);
  EXPECT_EQ(is31.size(), 1u);
  EXPECT_NE(is31.find(topic1), is31.end());
  EXPECT_EQ(is31[topic1].size(), 1u);
  EXPECT_EQ(*is31[topic1].begin(), time11);
}

TEST(TestForwardEstimator, reverse_order)
{
  // DAG is "A -> B -> C",
  // but PubInfo comes C -> B -> A
  auto fe = ForwardEstimator();
  const std::string topic1 = "topicA";
  const std::string topic2 = "topicB";
  const std::string topic3 = "topicC";

  // PubInfo of A
  const auto time11 = get_time(11, 110);
  auto pub_info11 = create_pubinfo(topic1, time11);

  // PubInfo of B
  const auto time21 = get_time(21, 210);
  auto pub_info21 = create_pubinfo(topic2, time21);
  add_input_info(pub_info21.get(), pub_info11.get());

  // PubInfo of C
  const auto time31 = get_time(31, 310);
  auto pub_info31 = create_pubinfo(topic3, time31);
  add_input_info(pub_info31.get(), pub_info21.get());

  // add PubInfo in C -> B -> A order
  fe.add(std::move(pub_info31));
  fe.add(std::move(pub_info21));
  fe.add(std::move(pub_info11));

  auto is31 = fe.get_input_sources(topic3, time31);
  EXPECT_EQ(is31.size(), 1u);
  EXPECT_NE(is31.find(topic1), is31.end());
  EXPECT_EQ(is31[topic1].size(), 1u);
  EXPECT_EQ(*is31[topic1].begin(), time11);
}

TEST(TestForwardEstimator, reverse_order_with_merge)
{
  // DAG
  //   1 --> 3 --> 4
  //       /
  //   2 --
  //
  // PubInfo order: 4 -> 3 -> 1 -> 2

  auto fe = ForwardEstimator();

  const std::string topic1 = "topic1";
  auto time11 = get_time(11, 110);
  const std::string topic2 = "topic2";
  auto time21 = get_time(21, 210);
  const std::string topic3 = "topic3";
  auto time31 = get_time(31, 310);
  const std::string topic4 = "topic4";
  auto time41 = get_time(41, 410);

  auto pub_info11 = create_pubinfo(topic1, time11);
  auto pub_info21 = create_pubinfo(topic2, time21);
  auto pub_info31 = create_pubinfo(topic3, time31);
  add_input_info(pub_info31.get(), pub_info11.get());
  add_input_info(pub_info31.get(), pub_info21.get());
  auto pub_info41 = create_pubinfo(topic4, time41);
  add_input_info(pub_info41.get(), pub_info31.get());

  // 4 -> 3 -> 1 -> 2
  fe.add(std::move(pub_info41));
  fe.add(std::move(pub_info31));
  fe.add(std::move(pub_info11));
  fe.add(std::move(pub_info21));

  auto is41 = fe.get_input_sources(topic4, time41);
  EXPECT_EQ(is41.size(), 2u);
}

TEST(TestForwardEstimator, expire_at_the_same_time)
{
  // DAG
  // A -> B -> C
  // PubInfo: A -> B -> C
  // expire at the same time
  auto fe = ForwardEstimator();

  const std::string topic1 = "topic1";
  const std::string topic2 = "topic2";
  const std::string topic3 = "topic3";
  auto time = get_time(11, 110);

  auto pub_info11 = create_pubinfo(topic1, time);
  auto pub_info21 = create_pubinfo(topic2, time);
  add_input_info(pub_info21.get(), pub_info11.get());
  auto pub_info31 = create_pubinfo(topic3, time);
  add_input_info(pub_info31.get(), pub_info21.get());

  fe.add(std::move(pub_info11));
  fe.add(std::move(pub_info21));
  fe.add(std::move(pub_info31));

  EXPECT_EQ(fe.get_input_sources(topic3, time).size(), 1u);

  fe.delete_expired(time);

  EXPECT_EQ(fe.get_input_sources(topic3, time).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic2, time).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic1, time).size(), 0u);
}

TEST(TestForwardEstimator, expire_step_by_step)
{
  // DAG
  // A -> B -> C
  // PubInfo: A -> B -> C
  // expire from C to A
  auto fe = ForwardEstimator();

  const std::string topic1 = "topic1";
  const std::string topic2 = "topic2";
  const std::string topic3 = "topic3";
  auto time11 = get_time(11, 110);
  auto time21 = get_time(21, 210);
  auto time31 = get_time(31, 310);

  auto pub_info11 = create_pubinfo(topic1, time11);
  auto pub_info21 = create_pubinfo(topic2, time21);
  add_input_info(pub_info21.get(), pub_info11.get());
  auto pub_info31 = create_pubinfo(topic3, time31);
  add_input_info(pub_info31.get(), pub_info21.get());

  fe.add(std::move(pub_info11));
  fe.add(std::move(pub_info21));
  fe.add(std::move(pub_info31));

  EXPECT_EQ(fe.get_input_sources(topic3, time31).size(), 1u);
  EXPECT_EQ(fe.get_input_sources(topic2, time21).size(), 1u);
  EXPECT_EQ(fe.get_input_sources(topic1, time11).size(), 1u);

  fe.delete_expired(time11);
  EXPECT_EQ(fe.get_input_sources(topic3, time31).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic2, time21).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic1, time11).size(), 0u);

  // ForwardEstimator internal data is maintenanced
  fe.delete_expired(time21);
  EXPECT_EQ(fe.get_input_sources(topic3, time31).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic2, time21).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic1, time11).size(), 0u);

  fe.delete_expired(time31);
  EXPECT_EQ(fe.get_input_sources(topic3, time31).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic2, time21).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic1, time11).size(), 0u);

}

TEST(TestForwardEstimator, expire_step_by_step_skew)
{
  // DAG
  // A -> B -> C
  // PubInfo: A -> B -> C
  // expire from C to A
  auto fe = ForwardEstimator();

  const std::string topic1 = "topic1";
  const std::string topic2 = "topic2";
  const std::string topic3 = "topic3";
  // skew timestamp for controlling timeout
  auto time11 = get_time(31, 310);
  auto time21 = get_time(21, 210);
  auto time31 = get_time(11, 110);

  auto pub_info11 = create_pubinfo(topic1, time11);
  auto pub_info21 = create_pubinfo(topic2, time21);
  add_input_info(pub_info21.get(), pub_info11.get());
  auto pub_info31 = create_pubinfo(topic3, time31);
  add_input_info(pub_info31.get(), pub_info21.get());

  fe.add(std::move(pub_info11));
  fe.add(std::move(pub_info21));
  fe.add(std::move(pub_info31));

  EXPECT_EQ(fe.get_input_sources(topic3, time31).size(), 1u);
  EXPECT_EQ(fe.get_input_sources(topic2, time21).size(), 1u);
  EXPECT_EQ(fe.get_input_sources(topic1, time11).size(), 1u);

  fe.delete_expired(time31);

  EXPECT_EQ(fe.get_input_sources(topic3, time31).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic2, time21).size(), 1u);
  EXPECT_EQ(fe.get_input_sources(topic1, time11).size(), 1u);

  fe.delete_expired(time21);

  EXPECT_EQ(fe.get_input_sources(topic3, time31).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic2, time21).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic1, time11).size(), 1u);

  fe.delete_expired(time11);
  EXPECT_EQ(fe.get_input_sources(topic3, time31).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic2, time21).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic1, time11).size(), 0u);
}

TEST(TestForwardEstimator, get_oldest_sensor_stamp)
{
  // DAG
  // A, B -> C

  const std::string topic1 = "topicA";
  const std::string topic2 = "topicB";
  const std::string topic3 = "topicC";

  auto time11 = get_time(11, 110);
  auto pubinfo11 = create_pubinfo(topic1, time11);
  auto time12 = get_time(12, 120);
  auto pubinfo12 = create_pubinfo(topic1, time12);

  auto time21 = get_time(21, 110);
  auto pubinfo21 = create_pubinfo(topic2, time21);
  auto time22 = get_time(22, 120);
  auto pubinfo22 = create_pubinfo(topic2, time22);

  auto time31 = get_time(31, 310);
  auto pubinfo31 = create_pubinfo(topic3, time31);

  add_input_info(pubinfo31.get(), pubinfo11.get());
  add_input_info(pubinfo31.get(), pubinfo12.get());
  add_input_info(pubinfo31.get(), pubinfo21.get());
  add_input_info(pubinfo31.get(), pubinfo22.get());

  auto fe = ForwardEstimator();
  fe.add(std::move(pubinfo11));
  fe.add(std::move(pubinfo21));
  fe.add(std::move(pubinfo31));

  auto oldest = fe.get_oldest_sensor_stamp(topic3, time31);
  if(!oldest) {
    FAIL() << "nil oldest";
  } else {
    EXPECT_EQ(*oldest, rclcpp::Time(time11));
  }

  fe.delete_expired(time11);

  auto oldest_without_11 = fe.get_oldest_sensor_stamp(topic3, time31);
  if(!oldest_without_11) {
    FAIL() << "nil oldest_without_11";
  } else {
    EXPECT_EQ(*oldest_without_11, rclcpp::Time(time21));
  }

  fe.delete_expired(time21);

  auto oldest_without_21 = fe.get_oldest_sensor_stamp(topic3, time31);
  if(!oldest_without_21) {
    SUCCEED();
  } else {
    FAIL() << "zombie oldest_without_21";
  }
}
