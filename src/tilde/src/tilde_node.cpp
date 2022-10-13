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

#include "tilde/tilde_node.hpp"

#include <string>

using tilde::TildeNode;

TildeNode::TildeNode(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  init();
}

TildeNode::TildeNode(
  const std::string & node_name, const std::string & namespace_,
  const rclcpp::NodeOptions & options)
: Node(node_name, namespace_, options)
{
  init();
}

void TildeNode::init()
{
  steady_clock_.reset(new rclcpp::Clock(RCL_STEADY_TIME));
  this->declare_parameter<bool>("enable_tilde", true);

  this->get_parameter("enable_tilde", enable_tilde_);

  param_callback_handle_ =
    this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & parameters) {
      auto result = rcl_interfaces::msg::SetParametersResult();

      result.successful = true;
      for (const auto & parameter : parameters) {
        if (parameter.get_name() == "enable_tilde") {
          enable_tilde_ = parameter.as_bool();
          for (auto & [topic, pub] : tilde_pubs_) {
            (void)topic;
            pub->set_enable(enable_tilde_);
          }
        }
      }

      return result;
    });
}
