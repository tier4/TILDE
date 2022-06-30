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

#include "tilde/stee_sources_table.hpp"

using tilde::SteeSourcesTable;
using tilde_msg::msg::SteeSource;

class TestSteeSourcesTable : public ::testing::Test
{
public:
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }
};

rclcpp::Time get_time(int32_t sec, uint32_t nsec)
{
  return rclcpp::Time(sec, nsec);
}

void add_sources(const std::string & in_topic,
                 int32_t stamp_sec, uint32_t stamp_nsec,
                 int32_t steady_sec, uint32_t steady_nsec,
                 std::vector<SteeSource> & out)
{
  SteeSource s;
  s.topic = in_topic;
  s.stamp = get_time(stamp_sec, stamp_nsec);
  s.first_subscription_steady_time = get_time(steady_sec, steady_nsec);
  out.emplace_back(std::move(s));
}

TEST_F(TestSteeSourcesTable, set_and_get) {
  SteeSourcesTable table(100);

  int32_t base_sec = 100;
  uint32_t base_nsec = 0;
  int32_t base_sec_steady = 0;
  int32_t base_nsec_steady = 100;

  // system input
  SteeSourcesTable::SourcesMsg sources;
  add_sources("in", base_sec, base_nsec, base_sec_steady, base_nsec_steady, sources);

  // topic1 input
  int32_t delte1_sec = 10;
  table.set("topic1", get_time(base_sec + delte1_sec, base_nsec), sources);

  auto latest_sources = table.get_latest_sources();

  EXPECT_EQ(latest_sources.size(), 1u);
  EXPECT_EQ(latest_sources.begin()->first, "topic1");

  auto in_sources = latest_sources.begin()->second;
  EXPECT_EQ(in_sources.size(), 1u);

  auto in_source = in_sources[0];
  EXPECT_EQ(in_source.topic, "in");
  EXPECT_EQ(in_source.stamp,
            get_time(base_sec, base_nsec));
  EXPECT_EQ(in_source.first_subscription_steady_time,
            get_time(base_sec_steady, base_nsec_steady));
}

