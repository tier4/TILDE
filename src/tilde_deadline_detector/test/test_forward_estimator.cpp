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

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "tilde_msg/msg/message_tracking_tag.hpp"
#include "tilde_msg/msg/sub_topic_time_info.hpp"

#include "tilde_deadline_detector/forward_estimator.hpp"

using tilde_deadline_detector::ForwardEstimator;
using tilde_msg::msg::MessageTrackingTag;
using tilde_msg::msg::SubTopicTimeInfo;
using TimeMsg = builtin_interfaces::msg::Time;

TimeMsg get_time(int sec, int nsec)
{
  TimeMsg t;
  t.sec = sec;
  t.nanosec = nsec;
  return t;
}

std::unique_ptr<MessageTrackingTag>
create_message_tracking_tag(
  const std::string & topic,
  const TimeMsg & time)
{
  auto message_tracking_tag = std::make_unique<MessageTrackingTag>();
  message_tracking_tag->output_info.topic_name = topic;
  message_tracking_tag->output_info.has_header_stamp = true;
  message_tracking_tag->output_info.header_stamp = time;

  return message_tracking_tag;
}

void add_input_info(
  MessageTrackingTag * target_message_tracking_tag,
  const MessageTrackingTag * const in_message_tracking_tag)
{
  auto ii = SubTopicTimeInfo();
  ii.topic_name = in_message_tracking_tag->output_info.topic_name;
  ii.has_header_stamp = true;
  ii.header_stamp = in_message_tracking_tag->output_info.header_stamp;
  target_message_tracking_tag->input_infos.push_back(ii);
}

TEST(TestForwardEstimator, one_sensor)
{
  auto fe = ForwardEstimator();
  const std::string topic = "sensor";

  const auto time1 = get_time(10, 100);

  auto is0 = fe.get_input_sources(topic, time1);
  EXPECT_EQ(is0.size(), 0u);

  // msg 1
  auto message_tracking_tag1 = create_message_tracking_tag(topic, time1);
  fe.add(std::move(message_tracking_tag1));

  auto is1 = fe.get_input_sources(topic, time1);
  EXPECT_NE(is1.find(topic), is1.end());
  EXPECT_EQ(is1[topic].size(), 1u);
  EXPECT_EQ(*is1[topic].begin(), time1);

  // msg 2
  auto time2 = get_time(11, 110);
  auto message_tracking_tag2 = create_message_tracking_tag(topic, time2);

  fe.add(std::move(message_tracking_tag2));

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
  auto message_tracking_tag11 = create_message_tracking_tag(topic1, time11);

  fe.add(std::move(message_tracking_tag11));

  auto is11 = fe.get_input_sources(topic1, time11);
  EXPECT_NE(is11.find(topic1), is11.end());
  EXPECT_EQ(is11[topic1].size(), 1u);
  EXPECT_EQ(*is11[topic1].begin(), time11);

  // sensor2 msg1
  const std::string topic2 = "sensor2";
  const auto time21 = get_time(21, 210);
  auto message_tracking_tag21 = create_message_tracking_tag(topic2, time21);

  fe.add(std::move(message_tracking_tag21));

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
  auto message_tracking_tag11 = create_message_tracking_tag(topic1, time11);

  // sensor2 msg1
  const auto time21 = time11;
  auto message_tracking_tag21 = create_message_tracking_tag(topic2, time21);
  add_input_info(message_tracking_tag21.get(), message_tracking_tag11.get());

  // sensor3 msg1
  rclcpp::Time time31 = time11;
  auto message_tracking_tag31 = create_message_tracking_tag(topic3, time31);
  add_input_info(message_tracking_tag31.get(), message_tracking_tag21.get());

  fe.add(std::move(message_tracking_tag11));
  fe.add(std::move(message_tracking_tag21));
  fe.add(std::move(message_tracking_tag31));

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
  auto message_tracking_tag11 = create_message_tracking_tag(topic1, time11);

  // sensor2 msg1
  const auto time21 = get_time(21, 210);
  auto message_tracking_tag21 = create_message_tracking_tag(topic2, time21);
  add_input_info(message_tracking_tag21.get(), message_tracking_tag11.get());

  // sensor3 msg1
  const auto time31 = get_time(31, 310);
  auto message_tracking_tag31 = create_message_tracking_tag(topic3, time31);
  add_input_info(message_tracking_tag31.get(), message_tracking_tag21.get());

  fe.add(std::move(message_tracking_tag11));
  fe.add(std::move(message_tracking_tag21));
  fe.add(std::move(message_tracking_tag31));

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
  auto message_tracking_tag11 = create_message_tracking_tag(topic1, time11);

  // sensor2 msg1
  const auto time21 = get_time(21, 210);
  auto message_tracking_tag21 = create_message_tracking_tag(topic2, time21);

  // sensor2 msg2
  const auto time22 = get_time(22, 220);
  auto message_tracking_tag22 = create_message_tracking_tag(topic2, time22);

  // sensor3 msg1 consists of msg 11, 21, 22
  const auto time31 = get_time(31, 310);
  auto message_tracking_tag31 = create_message_tracking_tag(topic3, time31);
  add_input_info(message_tracking_tag31.get(), message_tracking_tag11.get());
  add_input_info(message_tracking_tag31.get(), message_tracking_tag21.get());
  add_input_info(message_tracking_tag31.get(), message_tracking_tag22.get());

  fe.add(std::move(message_tracking_tag11));
  fe.add(std::move(message_tracking_tag21));
  fe.add(std::move(message_tracking_tag22));
  fe.add(std::move(message_tracking_tag31));

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
  // but MessageTrackingTag comes C -> A -> B
  auto fe = ForwardEstimator();
  const std::string topic1 = "topicA";
  const std::string topic2 = "topicB";
  const std::string topic3 = "topicC";

  // MessageTrackingTag of A
  const auto time11 = get_time(11, 110);
  auto message_tracking_tag11 = create_message_tracking_tag(topic1, time11);

  // MessageTrackingTag of B
  const auto time21 = get_time(21, 210);
  auto message_tracking_tag21 = create_message_tracking_tag(topic2, time21);
  add_input_info(message_tracking_tag21.get(), message_tracking_tag11.get());

  // MessageTrackingTag of C
  const auto time31 = get_time(31, 310);
  auto message_tracking_tag31 = create_message_tracking_tag(topic3, time31);
  add_input_info(message_tracking_tag31.get(), message_tracking_tag21.get());

  // add MessageTrackingTag in C -> A -> B order
  fe.add(std::move(message_tracking_tag31));
  fe.add(std::move(message_tracking_tag11));
  fe.add(std::move(message_tracking_tag21));

  auto is31 = fe.get_input_sources(topic3, time31);
  EXPECT_EQ(is31.size(), 1u);
  EXPECT_NE(is31.find(topic1), is31.end());
  EXPECT_EQ(is31[topic1].size(), 1u);
  EXPECT_EQ(*is31[topic1].begin(), time11);
}

TEST(TestForwardEstimator, reverse_order)
{
  // DAG is "A -> B -> C",
  // but MessageTrackingTag comes C -> B -> A
  auto fe = ForwardEstimator();
  const std::string topic1 = "topicA";
  const std::string topic2 = "topicB";
  const std::string topic3 = "topicC";

  // MessageTrackingTag of A
  const auto time11 = get_time(11, 110);
  auto message_tracking_tag11 = create_message_tracking_tag(topic1, time11);

  // MessageTrackingTag of B
  const auto time21 = get_time(21, 210);
  auto message_tracking_tag21 = create_message_tracking_tag(topic2, time21);
  add_input_info(message_tracking_tag21.get(), message_tracking_tag11.get());

  // MessageTrackingTag of C
  const auto time31 = get_time(31, 310);
  auto message_tracking_tag31 = create_message_tracking_tag(topic3, time31);
  add_input_info(message_tracking_tag31.get(), message_tracking_tag21.get());

  // add MessageTrackingTag in C -> B -> A order
  fe.add(std::move(message_tracking_tag31));
  fe.add(std::move(message_tracking_tag21));
  fe.add(std::move(message_tracking_tag11));

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
  // MessageTrackingTag order: 4 -> 3 -> 1 -> 2

  auto fe = ForwardEstimator();

  const std::string topic1 = "topic1";
  auto time11 = get_time(11, 110);
  const std::string topic2 = "topic2";
  auto time21 = get_time(21, 210);
  const std::string topic3 = "topic3";
  auto time31 = get_time(31, 310);
  const std::string topic4 = "topic4";
  auto time41 = get_time(41, 410);

  auto message_tracking_tag11 = create_message_tracking_tag(topic1, time11);
  auto message_tracking_tag21 = create_message_tracking_tag(topic2, time21);
  auto message_tracking_tag31 = create_message_tracking_tag(topic3, time31);
  add_input_info(message_tracking_tag31.get(), message_tracking_tag11.get());
  add_input_info(message_tracking_tag31.get(), message_tracking_tag21.get());
  auto message_tracking_tag41 = create_message_tracking_tag(topic4, time41);
  add_input_info(message_tracking_tag41.get(), message_tracking_tag31.get());

  // 4 -> 3 -> 1 -> 2
  fe.add(std::move(message_tracking_tag41));
  fe.add(std::move(message_tracking_tag31));
  fe.add(std::move(message_tracking_tag11));
  fe.add(std::move(message_tracking_tag21));

  auto is41 = fe.get_input_sources(topic4, time41);
  EXPECT_EQ(is41.size(), 2u);
}

TEST(TestForwardEstimator, expire_at_the_same_time)
{
  // DAG
  // A -> B -> C
  // MessageTrackingTag: A -> B -> C
  // expire at the same time
  auto fe = ForwardEstimator();

  const std::string topic1 = "topic1";
  const std::string topic2 = "topic2";
  const std::string topic3 = "topic3";
  auto time = get_time(11, 110);

  auto message_tracking_tag11 = create_message_tracking_tag(topic1, time);
  auto message_tracking_tag21 = create_message_tracking_tag(topic2, time);
  add_input_info(message_tracking_tag21.get(), message_tracking_tag11.get());
  auto message_tracking_tag31 = create_message_tracking_tag(topic3, time);
  add_input_info(message_tracking_tag31.get(), message_tracking_tag21.get());

  fe.add(std::move(message_tracking_tag11));
  fe.add(std::move(message_tracking_tag21));
  fe.add(std::move(message_tracking_tag31));

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
  // MessageTrackingTag: A -> B -> C
  // expire from C to A
  auto fe = ForwardEstimator();

  const std::string topic1 = "topic1";
  const std::string topic2 = "topic2";
  const std::string topic3 = "topic3";
  auto time11 = get_time(11, 110);
  auto time21 = get_time(21, 210);
  auto time31 = get_time(31, 310);

  auto message_tracking_tag11 = create_message_tracking_tag(topic1, time11);
  auto message_tracking_tag21 = create_message_tracking_tag(topic2, time21);
  add_input_info(message_tracking_tag21.get(), message_tracking_tag11.get());
  auto message_tracking_tag31 = create_message_tracking_tag(topic3, time31);
  add_input_info(message_tracking_tag31.get(), message_tracking_tag21.get());

  fe.add(std::move(message_tracking_tag11));
  fe.add(std::move(message_tracking_tag21));
  fe.add(std::move(message_tracking_tag31));

  EXPECT_EQ(fe.get_input_sources(topic3, time31).size(), 1u);
  EXPECT_EQ(fe.get_input_sources(topic2, time21).size(), 1u);
  EXPECT_EQ(fe.get_input_sources(topic1, time11).size(), 1u);

  fe.delete_expired(time11);
  EXPECT_EQ(fe.get_input_sources(topic3, time31).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic2, time21).size(), 0u);
  EXPECT_EQ(fe.get_input_sources(topic1, time11).size(), 0u);

  // ForwardEstimator internal data is maintained
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
  // MessageTrackingTag: A -> B -> C
  // expire from C to A
  auto fe = ForwardEstimator();

  const std::string topic1 = "topic1";
  const std::string topic2 = "topic2";
  const std::string topic3 = "topic3";
  // skew timestamp for controlling timeout
  auto time11 = get_time(31, 310);
  auto time21 = get_time(21, 210);
  auto time31 = get_time(11, 110);

  auto message_tracking_tag11 = create_message_tracking_tag(topic1, time11);
  auto message_tracking_tag21 = create_message_tracking_tag(topic2, time21);
  add_input_info(message_tracking_tag21.get(), message_tracking_tag11.get());
  auto message_tracking_tag31 = create_message_tracking_tag(topic3, time31);
  add_input_info(message_tracking_tag31.get(), message_tracking_tag21.get());

  fe.add(std::move(message_tracking_tag11));
  fe.add(std::move(message_tracking_tag21));
  fe.add(std::move(message_tracking_tag31));

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
  auto message_tracking_tag11 = create_message_tracking_tag(topic1, time11);
  auto time12 = get_time(12, 120);
  auto message_tracking_tag12 = create_message_tracking_tag(topic1, time12);

  auto time21 = get_time(21, 110);
  auto message_tracking_tag21 = create_message_tracking_tag(topic2, time21);
  auto time22 = get_time(22, 120);
  auto message_tracking_tag22 = create_message_tracking_tag(topic2, time22);

  auto time31 = get_time(31, 310);
  auto message_tracking_tag31 = create_message_tracking_tag(topic3, time31);

  add_input_info(message_tracking_tag31.get(), message_tracking_tag11.get());
  add_input_info(message_tracking_tag31.get(), message_tracking_tag12.get());
  add_input_info(message_tracking_tag31.get(), message_tracking_tag21.get());
  add_input_info(message_tracking_tag31.get(), message_tracking_tag22.get());

  auto fe = ForwardEstimator();
  fe.add(std::move(message_tracking_tag11));
  fe.add(std::move(message_tracking_tag21));
  fe.add(std::move(message_tracking_tag31));

  auto oldest = fe.get_oldest_sensor_stamp(topic3, time31);
  if (!oldest) {
    FAIL() << "nil oldest";
  } else {
    EXPECT_EQ(*oldest, rclcpp::Time(time11));
  }

  fe.delete_expired(time11);

  auto oldest_without_11 = fe.get_oldest_sensor_stamp(topic3, time31);
  if (!oldest_without_11) {
    FAIL() << "nil oldest_without_11";
  } else {
    EXPECT_EQ(*oldest_without_11, rclcpp::Time(time21));
  }

  fe.delete_expired(time21);

  auto oldest_without_21 = fe.get_oldest_sensor_stamp(topic3, time31);
  if (!oldest_without_21) {
    SUCCEED();
  } else {
    FAIL() << "zombie oldest_without_21";
  }
}

TEST(TestForwardEstimator, skip_topic)
{
  // DAG
  // A -> B -> C
  // skip B

  const std::string topic1 = "topicA";
  const std::string topic2 = "topicB";
  const std::string topic3 = "topicC";

  std::map<std::string, std::string> skip_out_to_in;
  skip_out_to_in[topic2] = topic1;

  auto time11 = get_time(11, 110);
  auto message_tracking_tag11 = create_message_tracking_tag(topic1, time11);

  // skipped message should have the same timestamp as input message
  auto message_tracking_tag21 = create_message_tracking_tag(topic2, time11);
  add_input_info(message_tracking_tag21.get(), message_tracking_tag11.get());

  auto time31 = get_time(31, 310);
  auto message_tracking_tag31 = create_message_tracking_tag(topic3, time31);
  add_input_info(message_tracking_tag31.get(), message_tracking_tag21.get());

  auto fe = ForwardEstimator();
  fe.set_skip_out_to_in(skip_out_to_in);

  fe.add(std::move(message_tracking_tag11));
  // ignore 21 to verify skip
  fe.add(std::move(message_tracking_tag31));

  auto input_sources31 = fe.get_input_sources(
    topic3, time31);

  EXPECT_EQ(input_sources31.size(), 1u);
  auto s = input_sources31.begin();
  EXPECT_EQ(s->first, topic1);
  EXPECT_EQ(s->second.size(), 1u);
  EXPECT_EQ(*s->second.begin(), time11);
}
