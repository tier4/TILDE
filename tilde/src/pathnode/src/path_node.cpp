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

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "pathnode/path_node.hpp"

using pathnode::PathNodeSubscriptionOptions;
using pathnode::PathNodeInfo;
using pathnode::PathNode;

PathNodeSubscriptionOptions::PathNodeSubscriptionOptions()
: valid_min_(0, 0), valid_max_(0, 0)
{
}

PathNodeInfo::PathNodeInfo()
: CLOCK_TYPE(RCL_SYSTEM_TIME),
  valid_min_(0, 0), valid_max_(0, 0)
{
}

PathNode::PathNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: Node(node_name, options),
  CLOCK_TYPE(RCL_SYSTEM_TIME)
{
}

PathNode::PathNode(
  const std::string & node_name,
  const std::string & namespace_,
  const rclcpp::NodeOptions & options)
: Node(node_name, namespace_, options),
  CLOCK_TYPE(RCL_SYSTEM_TIME)
{
}

PathNode::~PathNode()
{
}

void PathNode::setup_path(
  const PathNodeSubscriptionOptions & path_node_options)
{
  const auto & path_name = path_node_options.path_name_;

  std::lock_guard lock{path_node_info_map_mutex_};
  // if path_info already exists, then do nothing
  if (path_node_info_map_.find(path_name) != path_node_info_map_.end()) {
    return;
  }

  // prepare path_info data structure
  // if first node then prepare path_info publisher else prepare path_info callback
  auto info = std::make_shared<PathNodeInfo>();
  rclcpp::QoS qos(1);

  info->path_name_ = path_name;
  info->is_first_ = path_node_options.is_first_;
  info->valid_min_ = path_node_options.valid_min_;
  info->valid_max_ = path_node_options.valid_max_;

  if (info->is_first_) {
    info->pub_ = this->create_publisher<path_info_msg::msg::PathInfo>(path_name + "_info", qos);
  } else {
    auto path_info_callback =
      [this](path_info_msg::msg::PathInfo::UniquePtr msg) -> void
      {
        std::lock_guard lock{path_node_info_map_mutex_};
        auto info_it = path_node_info_map_.find(msg->path_name);
        if (info_it == path_node_info_map_.end()) {
          std::cout << "path_info not found: " << msg->path_name << std::endl;
          return;
        }
        auto info = info_it->second;
        info->path_tickets_.insert(rclcpp::Time(msg->path_start, info->CLOCK_TYPE));
      };
    info->sub_ = this->create_subscription<path_info_msg::msg::PathInfo>(
      path_name + "_info", qos, path_info_callback);
  }

  std::cout << "create_path_info: " << path_name << std::endl;
  path_node_info_map_[path_name] = info;
}

void PathNode::on_pathed_subscription(const std::string & path_name)
{
  std::lock_guard lock{path_node_info_map_mutex_};
  auto info_it = this->path_node_info_map_.find(path_name);
  if (info_it == this->path_node_info_map_.end()) {
    return;
  }

  auto & info = info_it->second;

  if (!info->is_first_) {
    return;
  }

  auto m = std::make_unique<path_info_msg::msg::PathInfo>();
  m->path_name = path_name;
  auto nw = now();
  m->path_start = nw;
  info->path_tickets_.insert(nw);  // for myself
  info->pub_->publish(std::move(m));
}

bool PathNode::pop_path_start_time(const std::string & path, rclcpp::Time & out)
{
  std::lock_guard lock{path_node_info_map_mutex_};
  bool ret = false;
  auto path_node_it = path_node_info_map_.find(path);
  if (path_node_it == path_node_info_map_.end()) {
    std::cout << "pop_path_start_time cannot find path_node_info: " << std::endl;
    return ret;
  }
  auto info = path_node_it->second;

  auto & tickets = info->path_tickets_;

  auto nw = now();
  for (auto it = tickets.begin(); it != tickets.end(); ) {
    // if ticket is too old, remove it
    if (*it + info->valid_max_ < nw) {
      std::cout << "pop_path_start_time erase: " << it->nanoseconds() << std::endl;
      // TODO(y-okumura-isp): silently erase? Is it bettter to notify that there is too old ticket?
      it = tickets.erase(it);
      continue;
    }
    if (*it - info->valid_min_ < nw && nw < *it + info->valid_max_) {
      ret = true;
      out = *it;
      it = tickets.erase(it);
      break;
    }
    it++;
  }

  return ret;
}

rclcpp::Duration PathNode::get_path_valid_min(const std::string & path)
{
  // TODO(y-okumura-isp): need lock? initialize once and read only access
  // std::lock_guard lock{path_node_info_map_mutex_};
  auto path_node_it = path_node_info_map_.find(path);
  if (path_node_it == path_node_info_map_.end()) {
    std::cout << "get_path_valid_min: cannot find path_node_info: " << path << std::endl;
    return rclcpp::Duration(0, 0);
  }

  auto & info = path_node_it->second;
  return info->valid_min_;
}

rclcpp::Duration PathNode::get_path_valid_max(const std::string & path)
{
  // TODO(y-okumura-isp): need lock? initialize once and read only access
  // std::lock_guard lock{path_node_info_map_mutex_};
  auto path_node_it = path_node_info_map_.find(path);
  if (path_node_it == path_node_info_map_.end()) {
    std::cout << "get_path_valid_max: cannot find path_node_info: " << path << std::endl;
    return rclcpp::Duration(0, 0);
  }

  auto & info = path_node_it->second;
  return info->valid_max_;
}

rclcpp::Time PathNode::now() const
{
  return rclcpp::Clock(CLOCK_TYPE).now();
}
