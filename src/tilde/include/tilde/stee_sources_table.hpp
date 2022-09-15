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

#ifndef TILDE__STEE_SOURCES_TABLE_HPP_
#define TILDE__STEE_SOURCES_TABLE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tilde_msg/msg/stee_source.hpp"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace tilde
{

class SteeSourcesTable
{
public:
  // resolved topic name
  using TopicName = std::string;
  // header stamp
  using Stamp = rclcpp::Time;
  using SourcesMsg = std::vector<tilde_msg::msg::SteeSource>;
  using TopicSources = std::map<TopicName, SourcesMsg>;
  // main data structure
  using Sources = std::map<TopicName, std::map<Stamp, SourcesMsg>>;
  // implicit relation
  using Latest = std::map<TopicName, Stamp>;

  /// Constructor
  SteeSourcesTable(
    size_t default_max_stamps_per_topic,
    std::map<TopicName, size_t> max_stamps_per_topic = std::map<TopicName, size_t>());

  /// Set input sources.
  /**
   * \param[in] topic resolved topic name
   * \param[in] stamp message stamp
   * \@aram[in] sources_msg stee sources in the message
   */
  void set(const TopicName & topic, const Stamp & stamp, const SourcesMsg & sources_msg);

  /// Get the latest sources.
  /**
   * \param[in] topic resolved topic name
   * \return empty if no topic
   */
  TopicSources get_latest_sources() const;

  /// Get sources of the specific message
  /**
   * \param[in] topic resolved topic name
   * \param[in] stamp message stamp
   * \return empty if not found
   */
  SourcesMsg get_sources(const TopicName & topic, const Stamp & stamp) const;

private:
  /// default maximum number of stamps to Sources
  size_t default_max_stamps_per_topic_;
  /// maximum number of stamps to Sources
  std::map<TopicName, size_t> max_stamps_per_topic_;
  /// Sources
  Sources sources_;
  /// Implicit relation
  Latest latest_;
};

}  // namespace tilde

#endif  // TILDE__STEE_SOURCES_TABLE_HPP_
