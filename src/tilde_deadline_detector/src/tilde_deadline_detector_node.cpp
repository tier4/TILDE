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

#include <chrono>
#include <string>
#include <sstream>
#include <thread>

#include "rcutils/time.h"

#include "builtin_interfaces/msg/time.hpp"

#include "tilde_deadline_detector/tilde_deadline_detector_node.hpp"

using tilde_msg::msg::PubInfo;
using std::chrono::milliseconds;

namespace tilde_deadline_detector
{
std::string time2str(const builtin_interfaces::msg::Time & time)
{
  std::ostringstream ret;
  ret << std::to_string(time.sec);
  ret << ".";
  ret << std::setfill('0') << std::setw(9) << std::to_string(time.nanosec);
  return ret.str();
}

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

TildeDeadlineDetectorNode::TildeDeadlineDetectorNode(const rclcpp::NodeOptions & options)
    : Node("tilde_deadline_detector_node", options)
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

  declare_parameter<std::vector<std::string>>("sensor_topics", std::vector<std::string>{});
  std::vector<std::string> tmp_sensor_topics;
  get_parameter("sensor_topics", tmp_sensor_topics);
  sensor_topics_.insert(tmp_sensor_topics.begin(), tmp_sensor_topics.end());

  declare_parameter<std::vector<std::string>>("target_topics", std::vector<std::string>{});
  std::vector<std::string> tmp_target_topics;
  get_parameter("target_topics", tmp_target_topics);
  target_topics_.insert(tmp_target_topics.begin(), tmp_target_topics.end());

  expire_ms_ = declare_parameter<int64_t>("expire_ms", 3 * 1000);
  cleanup_ms_ = declare_parameter<int64_t>("cleanup_ms", 3 * 1000);

  // wait discovery done
  std::set<std::string> topics;
  while(topics.size() == 0) {
    RCLCPP_INFO(this->get_logger(), "wait discovery");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    topics = get_pub_info_topics();
  }

  for(const auto & ignore : ignores) {
    topics.erase(ignore);
  }

  rclcpp::QoS qos(5);
  qos.best_effort();

  for(const auto & topic: topics) {
    RCLCPP_INFO(this->get_logger(), "subscribe: %s", topic.c_str());
    auto sub = create_subscription<PubInfo>(
        topic, qos,
        std::bind(&TildeDeadlineDetectorNode::pubinfo_callback, this, std::placeholders::_1));
    subs_.push_back(sub);
  }

  timer_ = create_wall_timer(
      milliseconds(cleanup_ms_),
      [this]() -> void
      {
        auto t = this->now();
        this->fe.delete_expired(t - rclcpp::Duration::from_nanoseconds(RCUTILS_MS_TO_NS(expire_ms_)));
      });
}

void TildeDeadlineDetectorNode::pubinfo_callback(PubInfo::UniquePtr pubinfo)
{
  auto target = pubinfo->output_info.topic_name;
  auto stamp = pubinfo->output_info.header_stamp;

  bool is_sensor = (sensor_topics_.find(pubinfo->output_info.topic_name) != sensor_topics_.end());
  fe.add(std::move(pubinfo), is_sensor);

  if(!contains(target_topics_, target)) {
    return;
  }

  // fe.debug_print();

  auto is = fe.get_input_sources(target, stamp);

  // print report
  std::cout << target << ": " << time2str(stamp) << "\n";
  for(auto it : is) {
    std::cout << "  " << it.first << ": ";
    for(auto stmp : it.second) {
      std::cout << time2str(stmp) << ", ";
    }
    std::cout << "\n";
  }
  std::cout << std::endl;

  // TODO(y-okumura-isp) send warning to diagnostic
}

}  // namespace tilde_deadline_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tilde_deadline_detector::TildeDeadlineDetectorNode)
