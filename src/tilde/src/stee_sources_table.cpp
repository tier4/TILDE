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

#include <cassert>
#include <map>

#include "tilde/stee_sources_table.hpp"

using tilde::SteeSourcesTable;

SteeSourcesTable::SteeSourcesTable(
  size_t default_max_stamps_per_topic,
  std::map<SteeSourcesTable::TopicName, size_t> max_stamps_per_topic)
: default_max_stamps_per_topic_(default_max_stamps_per_topic),
  max_stamps_per_topic_(max_stamps_per_topic)
{
}

void SteeSourcesTable::set(
  const SteeSourcesTable::TopicName & topic,
  const SteeSourcesTable::Stamp & stamp,
  const SteeSourcesTable::SourcesMsg & sources_msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  assert(stamp.get_clock_type() == RCL_ROS_TIME);
  sources_[topic][stamp] = sources_msg;
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

SteeSourcesTable::TopicSources
SteeSourcesTable::get_latest_sources() const
{
  SteeSourcesTable::TopicSources ret;
  std::lock_guard<std::mutex> lock(mtx_);

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
  const SteeSourcesTable::TopicName & topic,
  const SteeSourcesTable::Stamp & stamp) const
{
  std::lock_guard<std::mutex> lock(mtx_);
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
