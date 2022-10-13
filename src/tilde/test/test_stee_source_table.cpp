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

#include "tilde/stee_sources_table.hpp"

#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

using tilde::SteeSourceCmp;
using tilde::SteeSourcesTable;
using tilde_msg::msg::SteeSource;

TEST(TestSteeSourceCmp, simple_test)
{
  SteeSourceCmp cmp;
  tilde_msg::msg::SteeSource lhs;
  tilde_msg::msg::SteeSource rhs;

  // topic order differs
  lhs.topic = "in1";
  rhs.topic = "in2";

  EXPECT_TRUE(cmp(lhs, rhs));
  EXPECT_FALSE(cmp(rhs, lhs));

  // same topic order
  rhs.topic = lhs.topic;
  EXPECT_FALSE(cmp(lhs, rhs));
  EXPECT_FALSE(cmp(rhs, lhs));

  // stamp.sec differs
  rhs.stamp.sec = 1;
  EXPECT_TRUE(cmp(lhs, rhs));
  EXPECT_FALSE(cmp(rhs, lhs));

  lhs.stamp.sec = rhs.stamp.sec;

  // stamp.nanosec differs
  rhs.stamp.nanosec = 1;
  EXPECT_TRUE(cmp(lhs, rhs));
  EXPECT_FALSE(cmp(rhs, lhs));

  lhs.stamp.nanosec = rhs.stamp.nanosec;

  // first_subscription_steady_time.sec differs
  rhs.first_subscription_steady_time.sec = 1;
  EXPECT_TRUE(cmp(lhs, rhs));
  EXPECT_FALSE(cmp(rhs, lhs));

  lhs.first_subscription_steady_time.sec = rhs.first_subscription_steady_time.sec;

  // first_subscription_steady_time.nanosec differs
  rhs.first_subscription_steady_time.nanosec = 1;
  EXPECT_TRUE(cmp(lhs, rhs));
  EXPECT_FALSE(cmp(rhs, lhs));
}

class TestSteeSourcesTable : public ::testing::Test
{
public:
  void SetUp() override {}

  void TearDown() override {}
};

rclcpp::Time get_time(int32_t sec, uint32_t nsec) { return rclcpp::Time(sec, nsec, RCL_ROS_TIME); }

void add_sources(
  const std::string & in_topic, const rclcpp::Time & stamp, const rclcpp::Time & steady,
  std::vector<SteeSource> & out)
{
  SteeSource s;
  s.topic = in_topic;
  s.stamp = stamp;
  s.first_subscription_steady_time = steady;
  out.emplace_back(std::move(s));
}

TEST_F(TestSteeSourcesTable, set_one_entry)
{
  SteeSourcesTable table(100);

  /**
   * node topology:
   *   "in" -> "topic1"
   */

  // in
  auto in_stamp = get_time(100, 0);
  auto in_steady = get_time(0, 10);
  SteeSourcesTable::SourcesMsg sources_msg;
  add_sources("in", in_stamp, in_steady, sources_msg);

  // topic1
  auto topic11_stamp = get_time(101, 0);
  table.set("topic1", topic11_stamp, sources_msg);

  auto check = [&in_stamp, &in_steady](const SteeSourcesTable::SourcesMsg & msg) -> void {
    EXPECT_EQ(msg.size(), 1u);

    auto in_source = msg[0];
    EXPECT_EQ(in_source.topic, "in");
    EXPECT_EQ(in_source.stamp, in_stamp);
    EXPECT_EQ(in_source.first_subscription_steady_time, in_steady);
  };

  // test for get_latest_sources
  {
    auto latest_sources = table.get_latest_sources();

    EXPECT_EQ(latest_sources.size(), 1u);
    EXPECT_EQ(latest_sources.begin()->first, "topic1");

    auto in_sources = latest_sources.begin()->second;
    check(in_sources);
  }

  // test for get_sources
  {
    auto in_sources = table.get_sources("topic1", topic11_stamp);
    EXPECT_EQ(in_sources.size(), 1u);

    check(in_sources);
  }

  // test for get_sources, not found cases
  {
    auto not_found1 = table.get_sources("topic2", topic11_stamp);
    EXPECT_EQ(not_found1.size(), 0u);

    auto stamp2 = get_time(0, 1);
    auto not_found2 = table.get_sources("topic1", stamp2);
    EXPECT_EQ(not_found2.size(), 0u);
  }
}

TEST_F(TestSteeSourcesTable, set_one_entry_multiple_source_topics)
{
  SteeSourcesTable table(100);

  /**
   * node topology:
   *   in1  --> topic1
   *   in2  /
   */

  SteeSourcesTable::SourcesMsg sources;
  auto in11_stamp = get_time(100, 0);
  auto in11_steady = get_time(0, 10);
  add_sources("in1", in11_stamp, in11_steady, sources);

  auto in21_stamp = get_time(100, 10);
  auto in21_steady = get_time(0, 20);
  add_sources("in2", in21_stamp, in21_steady, sources);

  // topic1 input
  auto topic11_stamp = get_time(110, 0);
  table.set("topic1", topic11_stamp, sources);

  auto check = [&in11_stamp, &in11_steady, &in21_stamp,
                &in21_steady](const SteeSourcesTable::SourcesMsg & msg) -> void {
    EXPECT_EQ(msg.size(), 2u);
    for (const auto & in_source : msg) {
      if (in_source.topic == "in1") {
        EXPECT_EQ(in_source.stamp, in11_stamp);
        EXPECT_EQ(in_source.first_subscription_steady_time, in11_steady);
      } else if (in_source.topic == "in2") {
        EXPECT_EQ(in_source.stamp, in21_stamp);
        EXPECT_EQ(in_source.first_subscription_steady_time, in21_steady);
      } else {
        FAIL() << "unknown source topic";
      }
    }
  };

  // test for get_latest_sources
  {
    auto latest_sources = table.get_latest_sources();

    EXPECT_EQ(latest_sources.size(), 1u);
    EXPECT_EQ(latest_sources.begin()->first, "topic1");

    auto in_sources = latest_sources.begin()->second;
    check(in_sources);
  }

  // test for get_sources
  {
    auto in_sources = table.get_sources("topic1", topic11_stamp);
    check(in_sources);
  }
}

TEST_F(TestSteeSourcesTable, source_topic_has_multiple_stamp)
{
  SteeSourcesTable table(100);

  /**
   * node topology:
   *   in1  --> topic1
   *
   * topic1 has multiple stamps.
   */

  // topic1 msg1 input
  SteeSourcesTable::SourcesMsg sources11;
  auto in11_stamp = get_time(110, 0);
  auto in11_steady = get_time(0, 11);
  add_sources("in1", in11_stamp, in11_steady, sources11);

  auto topic11_stamp = get_time(110, 11);
  table.set("topic1", topic11_stamp, sources11);

  // topic1 msg2 input
  SteeSourcesTable::SourcesMsg sources12;
  auto in12_stamp = get_time(120, 00);
  auto in12_steady = get_time(0, 12);
  add_sources("in1", in12_stamp, in12_steady, sources12);

  auto topic12_stamp = get_time(120, 12);
  table.set("topic1", topic12_stamp, sources12);

  // test for get_latest sources
  auto latest_sources = table.get_latest_sources();
  EXPECT_EQ(latest_sources.size(), 1u);
  EXPECT_EQ(latest_sources.begin()->first, "topic1");

  const auto & in_sources = latest_sources.begin()->second;
  EXPECT_EQ(in_sources.size(), 1u);
  const auto & in_source = *in_sources.begin();
  EXPECT_EQ(in_source.topic, "in1");
  EXPECT_EQ(in_source.stamp, in12_stamp);
  EXPECT_EQ(in_source.first_subscription_steady_time, in12_steady);
}

TEST_F(TestSteeSourcesTable, source_topic_has_multiple_stamp_skew)
{
  SteeSourcesTable table(100);

  /**
   * node topology:
   *   in1  --> topic1
   *
   * topic1 has two stamps, t11 and t12 where t11 < t12.
   * But message arrival order is t12 -> t11.
   * Thus t11 is the latest message (i.e. skew)
   */

  // topic1 msg2 input
  SteeSourcesTable::SourcesMsg sources12;
  auto in12_stamp = get_time(120, 00);
  auto in12_steady = get_time(0, 12);
  add_sources("in1", in12_stamp, in12_steady, sources12);

  auto topic12_stamp = get_time(120, 12);
  table.set("topic1", topic12_stamp, sources12);

  // topic1 msg1 input
  SteeSourcesTable::SourcesMsg sources11;
  auto in11_stamp = get_time(110, 0);
  auto in11_steady = get_time(0, 11);
  add_sources("in1", in11_stamp, in11_steady, sources11);

  auto topic11_stamp = get_time(110, 11);
  table.set("topic1", topic11_stamp, sources11);

  // test for get_latest sources
  auto latest_sources = table.get_latest_sources();
  EXPECT_EQ(latest_sources.size(), 1u);
  EXPECT_EQ(latest_sources.begin()->first, "topic1");

  const auto & in_sources = latest_sources.begin()->second;
  EXPECT_EQ(in_sources.size(), 1u);
  const auto & in_source = *in_sources.begin();
  EXPECT_EQ(in_source.topic, "in1");
  EXPECT_EQ(in_source.stamp, in11_stamp);
  EXPECT_EQ(in_source.first_subscription_steady_time, in11_steady);
}

TEST_F(TestSteeSourcesTable, stamp_deletion)
{
  SteeSourcesTable table(1);

  /**
   * node topology:
   *   in1  --> topic1
   *
   * topic1 has multiple stamps.
   */

  // topic1 msg1 input
  SteeSourcesTable::SourcesMsg sources11;
  auto in11_stamp = get_time(110, 0);
  auto in11_steady = get_time(0, 11);
  add_sources("in1", in11_stamp, in11_steady, sources11);

  auto topic11_stamp = get_time(110, 11);
  table.set("topic1", topic11_stamp, sources11);

  EXPECT_EQ(table.get_sources("topic1", topic11_stamp).size(), 1u);

  // topic1 msg2 input
  SteeSourcesTable::SourcesMsg sources12;
  auto in12_stamp = get_time(120, 00);
  auto in12_steady = get_time(0, 12);
  add_sources("in1", in12_stamp, in12_steady, sources12);

  auto topic12_stamp = get_time(120, 12);
  table.set("topic1", topic12_stamp, sources12);

  // check topic1 msg1 is purged
  EXPECT_EQ(table.get_sources("topic1", topic11_stamp).size(), 0u);
}

TEST_F(TestSteeSourcesTable, duplicated_stamp)
{
  SteeSourcesTable table(100);

  {
    SteeSourcesTable::SourcesMsg sources_msg;
    auto stamp = get_time(110, 0);
    auto steady = get_time(0, 11);
    tilde_msg::msg::SteeSource source;
    source.topic = "in";
    source.stamp = stamp;
    source.first_subscription_steady_time = steady;

    // set same stamp multiple times
    sources_msg.push_back(source);
    sources_msg.push_back(source);
    sources_msg.push_back(source);

    // set different source
    source.topic = "in1";
    sources_msg.push_back(source);

    table.set("topic", stamp, sources_msg);
  }

  // expect only two sources
  auto latest_sources = table.get_latest_sources()["topic"];
  EXPECT_EQ(latest_sources.size(), 2u);
  for (const auto & source : latest_sources) {
    if (source.topic == "in") {
      EXPECT_EQ(source.stamp.sec, 110);
      EXPECT_EQ(source.stamp.nanosec, 0u);
      EXPECT_EQ(source.first_subscription_steady_time.sec, 0);
      EXPECT_EQ(source.first_subscription_steady_time.nanosec, 11u);
    } else if (source.topic == "in1") {
      EXPECT_EQ(source.stamp.sec, 110);
      EXPECT_EQ(source.stamp.nanosec, 0u);
      EXPECT_EQ(source.first_subscription_steady_time.sec, 0);
      EXPECT_EQ(source.first_subscription_steady_time.nanosec, 11u);
    }
  }
}
