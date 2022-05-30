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

#ifndef TILDE_DEADLINE_DETECTOR__FORWARD_ESTIMATOR_HPP_
#define TILDE_DEADLINE_DETECTOR__FORWARD_ESTIMATOR_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "tilde_msg/msg/pub_info.hpp"

namespace tilde_deadline_detector
{

template<class C, class T>
bool contains(const C & cnt, const T & v)
{
  return cnt.find(v) != cnt.end();
}

class ForwardEstimator
{
public:
  using TopicName = std::string;
  using PubInfoMsg = tilde_msg::msg::PubInfo;
  using HeaderStamp = rclcpp::Time;
  using RefToSource = std::weak_ptr<PubInfoMsg>;

  /// sensor sources: [sensor_topic][sensor_header_stamp] = PubInfoMsg
  using Sources = std::map<TopicName, std::map<HeaderStamp, std::shared_ptr<PubInfoMsg>>>;
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

  ForwardEstimator();

  /// add PubInfo
  void add(std::unique_ptr<PubInfoMsg> pub_info, bool is_sensor=false);

  /// get all sensor time
  /**
   * \param topic_name Target topic name
   * \param stamp Target header stamp
   * \return sensor topic vs its header stamps
   */
  InputSources get_input_sources(
    const std::string & topic_name,
    const HeaderStamp & stamp) const;

  /// get the oldest sensor time
  /**
   * \param topic_name Target topic name
   * \param stamp Target header stamp
   * \return the oldest header.stamp of all sensors.
   *
   * Calcurated latency is best effort i.e.
   * when it cannot gather all sensor PubInfo,
   * it returns the longest latency in gathered PubInfo.
   */
  std::optional<rclcpp::Time> get_oldest_sensor_stamp(
      const std::string & topic_name,
      const HeaderStamp & stamp) const;

  /// delete old data
  /**
   * \param threas Time point to delete data whose stamp <= thres
   */
  void delete_expired(const rclcpp::Time & thres);

  void debug_print(bool verbose=false) const;

private:
  /// all shared_ptr<PubInfo> of sensors to control pointer life time
  Sources sources_;

  /// input sensor information of (topic vs stamp).
  MessageSources message_sources_;

  /// gather sensor topics of topics to know graph
  TopicVsSensors topic_sensors_;

  /// pending messages
  PendingMessages pending_messages_;

  void update_pending(std::shared_ptr<PubInfoMsg> pub_info);
};

}  // namespace tilde_deadline_detector

#endif  // TILDE_DEADLINE_DETECTOR__FORWARD_ESTIMATOR_HPP_
