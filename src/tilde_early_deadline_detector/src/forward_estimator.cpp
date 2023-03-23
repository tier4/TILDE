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

#include "tilde_early_deadline_detector/forward_estimator.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>

namespace tilde_early_deadline_detector
{

ForwardEstimator::ForwardEstimator() {}

void ForwardEstimator::set_skip_out_to_in(const std::map<std::string, std::string> & skip_out_to_in)
{
  skip_out_to_in_ = skip_out_to_in;
}

void ForwardEstimator::add(
  std::unique_ptr<MessageTrackingTagMsg> _message_tracking_tag, bool is_sensor)
{
  if (!_message_tracking_tag->output_info.has_header_stamp) {
    return;
  }

  std::shared_ptr<MessageTrackingTagMsg> message_tracking_tag = std::move(_message_tracking_tag);

  const auto & topic_name = message_tracking_tag->output_info.topic_name;
  const auto stamp = rclcpp::Time(message_tracking_tag->output_info.header_stamp);

  // no input => it may be sensor source
  if (is_sensor || message_tracking_tag->input_infos.size() == 0) {
    // TODO(y-okumura-isp): what if timer fires without no new input in explicit API case

    sources_[topic_name][stamp] = message_tracking_tag;
    message_sources_[topic_name][stamp].insert(
      std::weak_ptr<MessageTrackingTagMsg>(message_tracking_tag));
    topic_sensors_[topic_name].insert(topic_name);

    auto pending_messages_topic_it = pending_messages_.find(topic_name);
    if (pending_messages_topic_it == pending_messages_.end()) {
      return;
    }

    auto pending_messages_it = pending_messages_topic_it->second.find(stamp);
    if (pending_messages_it == pending_messages_topic_it->second.end()) {
      return;
    }

    for (auto it : pending_messages_it->second) {
      const auto & waited_topic = std::get<0>(it);
      const auto & waited_stamp = std::get<1>(it);
      const auto & input_sources = message_sources_[topic_name][stamp];
      const auto & input_source_topics = topic_sensors_[topic_name];

      message_sources_[waited_topic][waited_stamp].insert(
        input_sources.begin(), input_sources.end());
      topic_sensors_[waited_topic].insert(input_source_topics.begin(), input_source_topics.end());
    }

    pending_messages_topic_it->second.erase(pending_messages_it);
    // we keep pending_messages_[topic_name] because it is fixed size resources
    return;
  }

  // if some messages wait this message, then
  // input of these messages are changed
  std::set<Message> pending_messages = std::set<Message>();
  auto pending_messages_topic_it = pending_messages_.find(topic_name);
  if (pending_messages_topic_it != pending_messages_.end()) {
    auto pending_messages_it = pending_messages_topic_it->second.find(stamp);
    if (pending_messages_it != pending_messages_topic_it->second.end()) {
      pending_messages.merge(pending_messages_it->second);
      pending_messages_topic_it->second.erase(pending_messages_it);
    }
    // we keep pending_messages_[topic_name] because it is fixed size resources
  }
  // have input => get reference
  for (const auto & input : message_tracking_tag->input_infos) {
    // get sources of input
    if (!input.has_header_stamp) {
      continue;
    }

    auto out_to_in_it = skip_out_to_in_.find(input.topic_name);
    const auto & input_topic =
      out_to_in_it != skip_out_to_in_.end() ? out_to_in_it->second : input.topic_name;

    const auto & input_stamp = rclcpp::Time(input.header_stamp);
    const auto & input_source_topics = topic_sensors_[input_topic];

    // if input of this message lacks,
    // both this message and pending messages wait the input
    auto message_sources_it = message_sources_[input_topic].find(input_stamp);
    if (message_sources_it == message_sources_[input_topic].end()) {
      auto & waiters = pending_messages_[input_topic][input_stamp];
      waiters.insert({topic_name, stamp});
      waiters.insert(pending_messages.begin(), pending_messages.end());
      continue;
    }

    const auto & input_sources = message_sources_[input_topic][input_stamp];

    // register sources
    message_sources_[topic_name][stamp].insert(input_sources.begin(), input_sources.end());
    topic_sensors_[topic_name].insert(input_source_topics.begin(), input_source_topics.end());

    // pending messages also get sources
    for (const auto & wait : pending_messages) {
      const auto & wait_topic = std::get<0>(wait);
      const auto & wait_stamp = std::get<1>(wait);
      message_sources_[wait_topic][wait_stamp].insert(input_sources.begin(), input_sources.end());
      topic_sensors_[wait_topic].insert(input_source_topics.begin(), input_source_topics.end());
    }
  }
}

std::string _time2str(const builtin_interfaces::msg::Time & time)
{
  std::ostringstream ret;
  ret << std::to_string(time.sec);
  ret << ".";
  ret << std::setfill('0') << std::setw(9) << std::to_string(time.nanosec);
  return ret.str();
}

ForwardEstimator::RefToSources ForwardEstimator::get_ref_to_sources(
  const std::string & topic_name, const HeaderStamp & stamp) const
{
  RefToSources ret;
  auto message_sources_topic_it = message_sources_.find(topic_name);
  if (message_sources_topic_it == message_sources_.end()) {
    return ret;
  }

  auto stamps_sources_it = message_sources_topic_it->second.find(stamp);
  if (stamps_sources_it == message_sources_topic_it->second.end()) {
    return ret;
  }

  return stamps_sources_it->second;
}

ForwardEstimator::InputSources ForwardEstimator::get_input_sources(
  const std::string & topic_name, const HeaderStamp & stamp) const
{
  InputSources is;
  auto message_sources_topic_it = message_sources_.find(topic_name);
  if (message_sources_topic_it == message_sources_.end()) {
    // std::cout << topic_name << ": not found in message_sources_" << std::endl;
    return is;
  }

  auto stamps_sources_it = message_sources_topic_it->second.find(stamp);
  if (stamps_sources_it == message_sources_topic_it->second.end()) {
    /*
    std::cout << topic_name << ":"
              << _time2str(stamp) << ": not found in message_sources_"
              << std::endl;
    */
    return is;
  }

  for (auto & weak_src : stamps_sources_it->second) {
    auto src = weak_src.lock();
    if (!src) {
      // std::cout << topic_name << ":" << _time2str(stamp) << " source deleted" << std::endl;
      continue;
    }

    is[src->output_info.topic_name].insert(src->output_info.header_stamp);
  }

  return is;
}

std::optional<rclcpp::Time> ForwardEstimator::get_oldest_sensor_stamp(
  const std::string & topic_name, const HeaderStamp & stamp) const
{
  auto is = get_input_sources(topic_name, stamp);
  if (is.empty()) {
    return std::nullopt;
  }

  std::set<HeaderStamp> mins;
  for (auto & it : is) {
    mins.insert(*std::min_element(it.second.begin(), it.second.end()));
  }

  return *(std::min_element(mins.begin(), mins.end()));
}

void ForwardEstimator::delete_expired(const rclcpp::Time & threshold)
{
  // delete references
  for (auto & it : message_sources_) {
    auto & stamp_refs = it.second;
    for (auto stamp_refs_it = stamp_refs.begin(); stamp_refs_it != stamp_refs.end();) {
      if (threshold < stamp_refs_it->first) {
        break;
      }
      stamp_refs_it = stamp_refs.erase(stamp_refs_it);
    }
  }

  // delete sources
  for (auto & it : sources_) {
    auto & stamp_message_tracking_tag = it.second;
    for (auto stamp_message_tracking_tag_it = stamp_message_tracking_tag.begin();
         stamp_message_tracking_tag_it != stamp_message_tracking_tag.end();) {
      if (threshold < stamp_message_tracking_tag_it->first) {
        break;
      }
      stamp_message_tracking_tag_it->second.reset();
      stamp_message_tracking_tag_it =
        stamp_message_tracking_tag.erase(stamp_message_tracking_tag_it);
    }
  }

  // delete pending_messages
  for (auto & it : pending_messages_) {
    auto & stamp_messages = it.second;
    for (auto stamp_messages_it = stamp_messages.begin();
         stamp_messages_it != stamp_messages.end();) {
      if (threshold < stamp_messages_it->first) {
        break;
      }
      stamp_messages_it = stamp_messages.erase(stamp_messages_it);
    }
  }
}

void ForwardEstimator::debug_print(bool verbose) const
{
  if (verbose) {
    std::cout << "sources_: " << sources_.size() << std::endl;
    for (auto & it : sources_) {
      std::cout << "  " << it.first << ": " << it.second.size() << std::endl;
    }

    std::cout << "message_sources_: " << message_sources_.size() << std::endl;
    for (auto & it : message_sources_) {
      std::cout << "  " << it.first << ": " << it.second.size() << std::endl;
    }

    std::cout << "topic_sensors_: " << topic_sensors_.size() << std::endl;
    for (auto & it : topic_sensors_) {
      std::cout << "  " << it.first << ": " << it.second.size() << std::endl;
    }
  } else {
    auto n_sources = 0;
    for (auto & it : sources_) {
      n_sources += it.second.size();
    }
    auto n_message_sources = 0;
    for (auto & it : message_sources_) {
      n_message_sources += it.second.size();
    }
    auto n_topic_sensors = 0;
    for (auto & it : topic_sensors_) {
      n_topic_sensors += it.second.size();
    }

    std::cout << "sources: " << n_sources << " "
              << "message_sources: " << n_message_sources << " "
              << "topic_sensors: " << n_topic_sensors << std::endl;
  }
}

std::map<ForwardEstimator::TopicName, size_t> ForwardEstimator::get_pending_message_counts() const
{
  std::map<TopicName, size_t> ret;

  for (const auto & pending_message : pending_messages_) {
    ret[pending_message.first] = pending_message.second.size();
  }

  return ret;
}

}  // namespace tilde_deadline_detector
