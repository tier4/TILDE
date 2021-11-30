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

#include <memory>
#include <string>

#include "pathnode/timing_advertise_publisher.hpp"
#include "path_info_msg/msg/sub_topic_time_info.hpp"

using pathnode::TimingAdvertisePublisherBase;

rclcpp::Time pathnode::get_timestamp(rclcpp::Time t, ...)
{
  std::cout << "get rclcpp::Time t" << std::endl;
  return t;
}


TimingAdvertisePublisherBase::TimingAdvertisePublisherBase()
: MAX_SUB_CALLBACK_INFOS_(10)
{
}

void TimingAdvertisePublisherBase::set_input_info(
  const std::string & sub_topic,
  const std::shared_ptr<const InputInfo> p)
{
  input_infos_[sub_topic] = p;
}

void TimingAdvertisePublisherBase::add_explicit_input_info(
  const std::string & sub_topic,
  const rclcpp::Time & stamp)
{
  InputInfo info;
  auto it = explicit_sub_callback_infos_.find(sub_topic);
  if (it != explicit_sub_callback_infos_.end() &&
    it->second.find(stamp) != it->second.end())
  {
    info.sub_time = it->second[stamp];
  } else {
    info.sub_time = rclcpp::Time(0, 0);
  }

  info.has_header_stamp = true;
  info.header_stamp = stamp;
  explicit_input_infos_[sub_topic].push_back(info);
}

void TimingAdvertisePublisherBase::set_input_info(path_info_msg::msg::PubInfo & info_msg)
{
  info_msg.input_infos.clear();

  if (explicit_input_infos_.size() == 0) {
    info_msg.input_infos.resize(input_infos_.size());

    size_t i = 0;
    for (const auto &[topic, input_info] : input_infos_) {
      info_msg.input_infos[i].topic_name = topic;
      info_msg.input_infos[i].sub_time = input_info->sub_time;
      info_msg.input_infos[i].has_header_stamp = input_info->has_header_stamp;
      info_msg.input_infos[i].header_stamp = input_info->header_stamp;
      i++;
    }
  } else {
    for (const auto &[topic, input_infos] : explicit_input_infos_) {
      for (const auto & input_info : input_infos) {
        path_info_msg::msg::SubTopicTimeInfo info;
        info.topic_name = topic;
        info.sub_time = input_info.sub_time;
        info.has_header_stamp = input_info.has_header_stamp;
        info.header_stamp = input_info.header_stamp;
        info_msg.input_infos.push_back(info);
      }
    }
    explicit_input_infos_.clear();
  }
}

void TimingAdvertisePublisherBase::set_explicit_subtime(
  const std::string & sub_topic,
  const rclcpp::Time & header_stamp,
  const rclcpp::Time & sub_time)
{
  auto & header_stamp2sub_time = explicit_sub_callback_infos_[sub_topic];
  header_stamp2sub_time[header_stamp] = sub_time;

  while (header_stamp2sub_time.size() > MAX_SUB_CALLBACK_INFOS_) {
    header_stamp2sub_time.erase(header_stamp2sub_time.begin());
  }
}
