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

void ForwardEstimator::add(std::shared_ptr<PubInfoMsg> pub_info)
{
  if (!pub_info->output_info.has_header_stamp) {return;}

  const auto & topic_name = pub_info->output_info.topic_name;
  const auto stamp = rclcpp::Time(pub_info->output_info.header_stamp);

  // no input => it may be sensor source
  if (pub_info->input_infos.size() == 0) {
    // TODO(y-okumura-isp): what if timer fires without no new input in explicit API case

    sources_[topic_name][stamp] = pub_info;
    message_sources_[topic_name][stamp].insert(std::weak_ptr<PubInfoMsg>(pub_info));
    topic_sensors_[topic_name].insert(topic_name);
    return;
  }

  // have input => get reference
  for (const auto & input : pub_info->input_infos) {
    // get sources of input
    if (!input.has_header_stamp) {continue;}
    const auto & input_topic = input.topic_name;
    const auto & input_stamp = rclcpp::Time(input.header_stamp);
    const auto & input_source_topics = topic_sensors_[input_topic];
    const auto & input_sources = message_sources_[input_topic][input_stamp];

    // TODO(y-okumura-isp): what if input PubInfo is not recieved yet?

    // register sources
    message_sources_[topic_name][stamp].insert(input_sources.begin(), input_sources.end());
    topic_sensors_[topic_name].insert(
      input_source_topics.begin(),
      input_source_topics.end());
  }
}

ForwardEstimator::InputSources ForwardEstimator::get_input_sources(
  const std::string & topic_name,
  const HeaderStamp & stamp)
{
  InputSources is;
  if (message_sources_.find(topic_name) == message_sources_.end()) {
    return is;
  }

  auto stamps_sources = message_sources_[topic_name];
  if (stamps_sources.find(stamp) == stamps_sources.end()) {
    return is;
  }

  auto sources = stamps_sources[stamp];
  for (auto & wsrc : sources) {
    auto src = wsrc.lock();
    if (!src) {continue;}

    is[src->output_info.topic_name].insert(src->output_info.header_stamp);
  }

  return is;
}

}  // namespace tilde_deadline_detector
