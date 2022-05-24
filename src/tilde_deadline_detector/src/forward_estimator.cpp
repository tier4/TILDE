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

#include <memory>
#include <string>

#include "tilde_deadline_detector/forward_estimator.hpp"

namespace tilde_deadline_detector
{

ForwardEstimator::ForwardEstimator()
{}

void ForwardEstimator::add(std::shared_ptr<PubInfoMsg> pub_info, bool is_sensor)
{
  if (!pub_info->output_info.has_header_stamp) {return;}

  const auto & topic_name = pub_info->output_info.topic_name;
  const auto stamp = rclcpp::Time(pub_info->output_info.header_stamp);

  // no input => it may be sensor source
  if (is_sensor || pub_info->input_infos.size() == 0) {
    // TODO(y-okumura-isp): what if timer fires without no new input in explicit API case

    sources_[topic_name][stamp] = pub_info;
    message_sources_[topic_name][stamp].insert(std::weak_ptr<PubInfoMsg>(pub_info));
    topic_sensors_[topic_name].insert(topic_name);

    if(contains(pending_messages_, Message{topic_name, stamp})) {
      for(auto it : pending_messages_[Message{topic_name, stamp}]) {
        const auto & waited_topic = std::get<0>(it);
        const auto & waited_stamp = std::get<1>(it);
        const auto & input_sources = message_sources_[topic_name][stamp];
        const auto & input_source_topics = topic_sensors_[topic_name];

        message_sources_[waited_topic][waited_stamp].insert(input_sources.begin(), input_sources.end());
        topic_sensors_[waited_topic].insert(
            input_source_topics.begin(),
            input_source_topics.end());
      }
    }

    return;
  }

  // if some messages wait this message, then
  // input of these messages are changed
  std::set<Message> pendings = std::set<Message>();
  if(contains(pending_messages_, Message{topic_name, stamp})) {
    auto it = pending_messages_.find(Message{topic_name, stamp});
    pendings.merge(it->second);
    pending_messages_.erase(it);
  }

  // have input => get reference
  for (const auto & input : pub_info->input_infos) {
    // get sources of input
    if (!input.has_header_stamp) {continue;}
    const auto & input_topic = input.topic_name;
    const auto & input_stamp = rclcpp::Time(input.header_stamp);
    const auto & input_source_topics = topic_sensors_[input_topic];

    // if input of this message lacks,
    // both this message and pending messages wait the input
    if(!contains(message_sources_[input_topic], input_stamp)) {
      pending_messages_[{input_topic, input_stamp}].insert({topic_name, stamp});

      pending_messages_[{input_topic, input_stamp}].insert(
          pendings.begin(), pendings.end());
      continue;
    }

    const auto & input_sources = message_sources_[input_topic][input_stamp];

    // register sources
    message_sources_[topic_name][stamp].insert(input_sources.begin(), input_sources.end());
    topic_sensors_[topic_name].insert(
      input_source_topics.begin(),
      input_source_topics.end());

    // pending messages also get sources
    for(const auto & wait : pendings) {
        const auto & wait_topic = std::get<0>(wait);
        const auto & wait_stamp = std::get<1>(wait);
        message_sources_[wait_topic][wait_stamp].insert(input_sources.begin(), input_sources.end());
        topic_sensors_[wait_topic].insert(
            input_source_topics.begin(),
            input_source_topics.end());
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

ForwardEstimator::InputSources ForwardEstimator::get_input_sources(
  const std::string & topic_name,
  const HeaderStamp & stamp)
{
  InputSources is;
  if (message_sources_.find(topic_name) == message_sources_.end()) {
    std::cout << topic_name << ": not found in message_sources_" << std::endl;
    return is;
  }

  auto stamps_sources = message_sources_[topic_name];
  if (stamps_sources.find(stamp) == stamps_sources.end()) {
    std::cout << topic_name << ":" << _time2str(stamp) << ": not found in message_sources_" << std::endl;
    return is;
  }

  auto sources = stamps_sources[stamp];
  for (auto & wsrc : sources) {
    auto src = wsrc.lock();
    if (!src) {
      std::cout << topic_name << ":" << _time2str(stamp) << " source deleted" << std::endl;
      continue;
    }

    is[src->output_info.topic_name].insert(src->output_info.header_stamp);
  }

  return is;
}

void ForwardEstimator::debug_print() const
{
  std::cout << "sources_: " << sources_.size() << std::endl;
  for(auto it : sources_) {
    std::cout << "  " << it.first << ": " << it.second.size() << std::endl;
  }

  std::cout << "message_sources_: " << message_sources_.size() << std::endl;
  for(auto it : message_sources_) {
    std::cout << "  " << it.first << ": " << it.second.size() << std::endl;
  }

  std::cout << "topic_sensors_: " << topic_sensors_.size() << std::endl;
  for(auto it : topic_sensors_) {
    std::cout << "  " << it.first << ": " << it.second.size() << std::endl;
  }
}

}  // namespace tilde_deadline_detector
