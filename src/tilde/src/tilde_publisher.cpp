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

#include "tilde/tilde_publisher.hpp"
#include "tilde_msg/msg/sub_topic_time_info.hpp"

using tilde::TildePublisherBase;
using tilde::InputInfo;

bool InputInfo::operator==(const InputInfo & rhs) const
{
  return
    sub_time == rhs.sub_time &&
    sub_time_steady == rhs.sub_time_steady &&
    has_header_stamp == rhs.has_header_stamp &&
    header_stamp == rhs.header_stamp
  ;
}

rclcpp::Time tilde::get_timestamp(rclcpp::Time t, ...)
{
  std::cout << "get rclcpp::Time t" << std::endl;
  return t;
}


TildePublisherBase::TildePublisherBase(
  std::shared_ptr<rclcpp::Clock> clock,
  std::shared_ptr<rclcpp::Clock> steady_clock,
  const std::string & node_fqn)
: clock_(clock), steady_clock_(steady_clock),
  node_fqn_(node_fqn), seq_(0),
  is_explicit_(false),
  MAX_SUB_CALLBACK_INFOS_SEC_(2)
{
}

void TildePublisherBase::set_implicit_input_info(
  const std::string & sub_topic,
  const std::shared_ptr<const InputInfo> p)
{
  input_infos_[sub_topic] = p;
  if (sub_topics_.count(sub_topic) == 0) {
    sub_topics_[sub_topic] = sub_topic;
    tracepoint(
      TRACEPOINT_PROVIDER,
      tilde_subscribe_added,
      &sub_topics_[sub_topic],
      node_fqn_.c_str(),
      sub_topic.c_str()
    );
  }
}

void TildePublisherBase::add_explicit_input_info(
  const std::string & sub_topic,
  const rclcpp::Time & stamp)
{
  InputInfo info;

  if (!is_explicit_) {
    is_explicit_ = true;
  }

  // find sub callback time info and fill info
  auto it = explicit_sub_time_infos_.find(sub_topic);
  if (it != explicit_sub_time_infos_.end() &&
    it->second.find(stamp) != it->second.end())
  {
    info.sub_time = it->second[stamp]->sub_time;
    info.sub_time_steady = it->second[stamp]->sub_time_steady;
  }

  info.has_header_stamp = true;
  info.header_stamp = stamp;
  explicit_input_infos_[sub_topic].push_back(info);

  if (sub_topics_.count(sub_topic) == 0) {
    sub_topics_[sub_topic] = sub_topic;
    tracepoint(
      TRACEPOINT_PROVIDER,
      tilde_subscribe_added,
      &sub_topics_[sub_topic],
      node_fqn_.c_str(),
      sub_topic.c_str()
    );
  }
}

void TildePublisherBase::fill_input_info(tilde_msg::msg::MessageTrackingTag & info_msg)
{
  info_msg.input_infos.clear();

  if (!is_explicit_) {
    info_msg.input_infos.resize(input_infos_.size());

    size_t i = 0;
    for (const auto &[topic, input_info] : input_infos_) {
      info_msg.input_infos[i].topic_name = topic;
      info_msg.input_infos[i].sub_time = input_info->sub_time;
      info_msg.input_infos[i].sub_time_steady = input_info->sub_time_steady;
      info_msg.input_infos[i].has_header_stamp = input_info->has_header_stamp;
      info_msg.input_infos[i].header_stamp = input_info->header_stamp;
      i++;
    }
  } else {
    for (const auto &[topic, input_infos] : explicit_input_infos_) {
      for (const auto & input_info : input_infos) {
        tilde_msg::msg::SubTopicTimeInfo info;
        info.topic_name = topic;
        info.sub_time = input_info.sub_time;
        info.sub_time_steady = input_info.sub_time_steady;
        info.has_header_stamp = input_info.has_header_stamp;
        info.header_stamp = input_info.header_stamp;
        info_msg.input_infos.push_back(info);
      }
    }
    explicit_input_infos_.clear();
  }
}

void TildePublisherBase::set_explicit_subscription_time(
  const std::string & sub_topic,
  const std::shared_ptr<const InputInfo> p)
{
  auto & header_stamp2sub_time = explicit_sub_time_infos_[sub_topic];
  header_stamp2sub_time[p->header_stamp] = p;

  // cleanup old infos
  rclcpp::Duration dur(MAX_SUB_CALLBACK_INFOS_SEC_, 0);
  auto thres = clock_->now() - dur;

  for (auto it = header_stamp2sub_time.begin();
    it != header_stamp2sub_time.end(); )
  {
    if (it->first < thres) {
      it = header_stamp2sub_time.erase(it);
    } else {
      break;
    }
  }
}

void TildePublisherBase::set_max_sub_callback_infos_sec(size_t sec)
{
  MAX_SUB_CALLBACK_INFOS_SEC_ = sec;
}

bool TildePublisherBase::get_input_info(
  const std::string & topic,
  const rclcpp::Time & header_stamp,
  InputInfo & info)
{
  auto hs2ii_it = explicit_sub_time_infos_.find(topic);
  if (hs2ii_it == explicit_sub_time_infos_.end()) {
    return false;
  }

  auto header_stamp2input_info = hs2ii_it->second;
  auto it = header_stamp2input_info.find(header_stamp);
  if (it == header_stamp2input_info.end()) {
    return false;
  }

  info = *(it->second);
  return true;
}

void TildePublisherBase::print_input_infos()
{
  auto time2str =
    [](const rclcpp::Time & rt) -> std::string
    {
      builtin_interfaces::msg::Time t = rt;
      return std::string("sec: ") +
             std::to_string(t.sec) +
             " nsec: " +
             std::to_string(t.nanosec);
    };

  std::cout << "print_input_infos\n";
  for (auto & [topic, pinfo] : input_infos_) {
    std::cout << "  " << topic << "\n";
    std::cout << "  sub_time: " <<
      time2str(pinfo->sub_time) <<
      "\n  stamp: " <<
      time2str(pinfo->header_stamp) <<
      std::endl;
  }
}
