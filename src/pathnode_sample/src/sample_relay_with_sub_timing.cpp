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

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "pathnode/tilde_node.hpp"

using namespace std::chrono_literals;

namespace pathnode_sample
{
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class RelayWithSubTiming : public pathnode::TildeNode
{
public:
  explicit RelayWithSubTiming(const rclcpp::NodeOptions & options)
  : TildeNode("relay", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto callback =
      [this](std_msgs::msg::String::UniquePtr msg) -> void
      {
        std::cout << msg->data << std::endl;
        pub_->publish(std::move(msg));
      };

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    sub_ = this->create_tilde_subscription<std_msgs::msg::String>("in", qos, callback);
    pub_ = this->create_tilde_publisher<std_msgs::msg::String>("out", qos);
  }

private:
  std::shared_ptr<pathnode::TildePublisher<std_msgs::msg::String>> pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace pathnode_sample

RCLCPP_COMPONENTS_REGISTER_NODE(pathnode_sample::RelayWithSubTiming)
