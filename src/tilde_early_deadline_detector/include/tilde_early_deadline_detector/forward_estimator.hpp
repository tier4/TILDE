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

#ifndef TILDE_EARLY_DEADLINE_DETECTOR__FORWARD_ESTIMATOR_HPP_
#define TILDE_EARLY_DEADLINE_DETECTOR__FORWARD_ESTIMATOR_HPP_

#include <chrono>
#include <map>
#include <memory>
// NOLINT to prevent Found C system header after C++ system header
#include "rclcpp/rclcpp.hpp"
#include "tilde_msg/msg/message_tracking_tag.hpp"

#include <optional>  // NOLINT
#include <set>
#include <string>
#include <tuple>
#include <unordered_set>

namespace tilde_early_deadline_detector
{

template <class C, class T>
bool contains(const C & cnt, const T & v)
{
  return cnt.find(v) != cnt.end();
}

class ForwardEstimator
{
public:
  using TopicName = std::string;
  using MessageTrackingTagMsg = tilde_msg::msg::MessageTrackingTag;
  using HeaderStamp = rclcpp::Time;
  using RefToSource = std::weak_ptr<MessageTrackingTagMsg>;

  /// sensor sources: [sensor_topic][sensor_header_stamp] = MessageTrackingTagMsg
  using Sources =
    std::map<TopicName, std::map<HeaderStamp, std::shared_ptr<MessageTrackingTagMsg>>>;
  /// input sensor topics of the target topic
  using TopicVsSensors = std::map<TopicName, std::set<TopicName>>;
  /// to know sources
  using RefToSources = std::set<RefToSource, std::owner_less<RefToSource>>;
  /// sources of the specific message
  // if target topic is sensor, MessageInputs[topic][stamp] points itself source.
  using MessageSources = std::map<TopicName, std::map<HeaderStamp, RefToSources>>;
  /// input sources which output consists of
  using InputSources = std::map<TopicName, std::set<HeaderStamp>>;
  /// pending messages: <wanted message>: <pending messages>
  using Message = std::tuple<TopicName, HeaderStamp>;
  using PendingMessages = std::map<TopicName, std::map<HeaderStamp, std::set<Message>>>;

  /// Constructor
  ForwardEstimator();

  /// skip_out_to_in_ setter
  /**
   * \param skip_out_to_in skip topic setting
   */
  void set_skip_out_to_in(const std::map<std::string, std::string> & skip_out_to_in);

  /// add MessageTrackingTag
  void add(std::unique_ptr<MessageTrackingTagMsg> message_tracking_tag, bool is_sensor = false);

  /// get sources of give message
  /**
   * \param topic_name Target topic name
   * \param stamp Target header stamp
   * \return set of references to sources
   */
  RefToSources get_ref_to_sources(const std::string & topic_name, const HeaderStamp & stamp) const;

  /// get all sensor time
  /**
   * \param topic_name Target topic name
   * \param stamp Target header stamp
   * \return sensor topic vs its header stamps
   */
  InputSources get_input_sources(const std::string & topic_name, const HeaderStamp & stamp) const;

  /// get the oldest sensor time
  /**
   * \param topic_name Target topic name
   * \param stamp Target header stamp
   * \return the oldest header.stamp of all sensors.
   *
   * Calculated latency is best effort i.e.
   * when it cannot gather all sensor MessageTrackingTag,
   * it returns the longest latency in gathered MessageTrackingTag.
   */
  std::optional<rclcpp::Time> get_oldest_sensor_stamp(
    const std::string & topic_name, const HeaderStamp & stamp) const;

  /// delete old data
  /**
   * \param threshold Time point to delete data whose stamp <= threshold
   */
  void delete_expired(const rclcpp::Time & threshold);

  void debug_print(bool verbose = false) const;

  /// get pending message counts
  /**
   * \return pending topic name vs the number of waited stamps.
   */
  std::map<TopicName, size_t> get_pending_message_counts() const;

private:
  /// all shared_ptr<MessageTrackingTag> of sensors to control pointer life time
  Sources sources_;

  /// skip topic setting
  std::map<std::string, std::string> skip_out_to_in_;

  /// input sensor information of (topic vs stamp).
  MessageSources message_sources_;

  /// gather sensor topics of topics to know graph
  TopicVsSensors topic_sensors_;

  /// pending messages
  PendingMessages pending_messages_;

  void update_pending(std::shared_ptr<MessageTrackingTagMsg> message_tracking_tag);
};

}  // namespace tilde_deadline_detector

#endif  // TILDE_EARLY_DEADLINE_DETECTOR__FORWARD_ESTIMATOR_HPP_
