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

#include <string>

#include "tilde/stee_node.hpp"

using tilde::SteeNode;

SteeNode::SteeNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  init();
}

SteeNode::SteeNode(
  const std::string & node_name,
  const std::string & namespace_,
  const rclcpp::NodeOptions & options)
: Node(node_name, namespace_, options)
{
  init();
}

SteeNode::~SteeNode()
{
}

void SteeNode::init()
{
  steady_clock_.reset(new rclcpp::Clock(RCL_STEADY_TIME));
  // TODO(y-okumura-isp): set appropriate max stamps
  source_table_.reset(new SteeSourcesTable(100));

  auto stop_topics_vec = declare_parameter<std::vector<std::string>>(
      "stee_stop_topics", std::vector<std::string>{
        "/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias",
      });
  for (const auto & t : stop_topics_vec) {
    stop_topics_.insert(t);
  }
}
