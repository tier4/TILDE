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

#include <string>

#include "tilde_deadline_detector/tilde_deadline_detector_node.hpp"

using tilde_msg::msg::PubInfo;

namespace tilde_deadline_detector
{
TildeDeadlineDetectorNode::TildeDeadlineDetectorNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  init();
}

TildeDeadlineDetectorNode::TildeDeadlineDetectorNode(
  const std::string & node_name,
  const std::string & namespace_,
  const rclcpp::NodeOptions & options)
: Node(node_name, namespace_, options)
{
  init();
}

TildeDeadlineDetectorNode::~TildeDeadlineDetectorNode()
{}

std::set<std::string> TildeDeadlineDetectorNode::get_pub_info_topics() const
{
  std::set<std::string> ret;

  const std::string msg_type = "tilde_msg/msg/PubInfo";
  auto topic_and_types = get_topic_names_and_types();
  for(const auto &it : topic_and_types) {
    if(std::find(it.second.begin(), it.second.end(), msg_type) == it.second.end()) {
      continue;
    }
    ret.insert(it.first);
  }

  return ret;
}

void TildeDeadlineDetectorNode::init()
{
  declare_parameter<std::vector<std::string>>("ignore_topics", std::vector<std::string>{});
  std::vector<std::string> ignores;
  get_parameter("ignore_topics", ignores);

  auto topics = get_pub_info_topics();
  for(const auto & ignore : ignores) {
    topics.erase(ignore);
  }

  for(const auto & topic: topics) {
    auto sub = create_subscription<PubInfo>(
        topic, rclcpp::QoS(1),
        std::bind(&TildeDeadlineDetectorNode::pubinfo_callback, this, std::placeholders::_1));
    subs_.push_back(sub);
  }
}

void TildeDeadlineDetectorNode::pubinfo_callback(const PubInfo::SharedPtr msg)
{
  
}

}  // namespace tilde_deadline_detector
