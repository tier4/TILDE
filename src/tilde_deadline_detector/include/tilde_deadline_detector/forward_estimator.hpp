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
#include <set>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "tilde_msg/msg/pub_info.hpp"

namespace tilde_deadline_detector
{
class ForwardEstimator
{
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

public:
  ForwardEstimator();

  /// add PubInfo
  void add(std::shared_ptr<PubInfoMsg> pub_info);

  /// get latency from the oldest input sensor
  /**
   * \param topic_name Target topic name
   * \param stamp Target header stamp
   * \return latency in ms. -1 if not able to calcurate.
   *
   * Calcurated latency is best effort i.e.
   * when it cannot gather all sensor PubInfo,
   * it returns the longest latency in gathered PubInfo.
   */
  float get_latency_ms(const std::string & topic_name,
                       const HeaderStamp & stamp);

  /// garbage collect
  void gc();

private:
  /// all shared_ptr<PubInfo> of sensors to control pointer life time
  Sources sources_;

  /// input sensor information of (topic vs stamp).
  MessageSources message_sources_;

  /// gather sensor topics of topics to know graph
  TopicVsSensors topic_sensors_;
};

}  // namespace tilde_deadline_detector

#endif  // TILDE_DEADLINE_DETECTOR__FORWARD_ESTIMATOR_HPP_
