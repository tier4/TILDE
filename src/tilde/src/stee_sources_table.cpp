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

#include <cassert>
#include <map>

using tilde::SteeSourceCmp;
using tilde::SteeSourcesTable;

SteeSourcesTable::SteeSourcesTable(
  size_t default_max_stamps_per_topic,
  std::map<SteeSourcesTable::TopicName, size_t> max_stamps_per_topic)
: default_max_stamps_per_topic_(default_max_stamps_per_topic),
  max_stamps_per_topic_(max_stamps_per_topic)
{
}

bool SteeSourceCmp::operator()(
  const tilde_msg::msg::SteeSource & lhs, const tilde_msg::msg::SteeSource & rhs) const
{
  return (
    lhs.topic < rhs.topic || lhs.stamp.sec < rhs.stamp.sec ||
    lhs.stamp.nanosec < rhs.stamp.nanosec ||
    lhs.first_subscription_steady_time.sec < rhs.first_subscription_steady_time.sec ||
    lhs.first_subscription_steady_time.nanosec < rhs.first_subscription_steady_time.nanosec);
}

void SteeSourcesTable::set(
  const SteeSourcesTable::TopicName & topic, const SteeSourcesTable::Stamp & stamp,
  const SteeSourcesTable::SourcesMsg & sources_msg)
{
  assert(stamp.get_clock_type() == RCL_ROS_TIME);

  std::set<tilde_msg::msg::SteeSource, SteeSourceCmp> found;
  for (const auto & source : sources_msg) {
    if (found.find(source) != found.end()) {
      continue;
    }
    found.insert(source);
    sources_[topic][stamp].push_back(source);
  }
  latest_[topic] = stamp;

  auto max_stamps = default_max_stamps_per_topic_;
  auto max_it = max_stamps_per_topic_.find(topic);
  if (max_it != max_stamps_per_topic_.end()) {
    max_stamps = max_it->second;
  }

  auto & stamp_vs_sources = sources_[topic];
  while (stamp_vs_sources.size() > max_stamps) {
    stamp_vs_sources.erase(stamp_vs_sources.begin());
  }
}

SteeSourcesTable::TopicSources SteeSourcesTable::get_latest_sources() const
{
  SteeSourcesTable::TopicSources ret;

  for (const auto & latest_it : latest_) {
    const auto & topic = latest_it.first;
    const auto & stamp = latest_it.second;

    auto sources_it = sources_.find(topic);
    if (sources_it == sources_.end()) {
      continue;
    }

    const auto & stamp_sources = sources_it->second;
    auto stamp_sources_it = stamp_sources.find(stamp);
    if (stamp_sources_it != stamp_sources.end()) {
      const auto & sources = stamp_sources_it->second;
      ret[topic] = sources;
    }
  }

  return ret;
}

SteeSourcesTable::SourcesMsg SteeSourcesTable::get_sources(
  const SteeSourcesTable::TopicName & topic, const SteeSourcesTable::Stamp & stamp) const
{
  assert(stamp.get_clock_type() == RCL_ROS_TIME);
  SteeSourcesTable::SourcesMsg empty;

  auto it = sources_.find(topic);
  if (it == sources_.end()) {
    return empty;
  }

  auto it_stamp_vs_sources = it->second.find(stamp);
  if (it_stamp_vs_sources == it->second.end()) {
    return empty;
  }

  return it_stamp_vs_sources->second;
}
