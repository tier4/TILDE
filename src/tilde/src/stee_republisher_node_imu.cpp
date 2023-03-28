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

#include "tilde/stee_node.hpp"

using sensor_msgs::msg::Imu;
using tilde_msg::msg::SteeImu;

namespace tilde
{

/// Stee Republisher
// TODO(y-okumura-isp): support other message types than Imu
class SteeRepublisherNodeImu : public SteeNode
{
public:
  explicit SteeRepublisherNodeImu(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SteeNode("stee_republisher_node", options)
  {
    init();
  }

  explicit SteeRepublisherNodeImu(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SteeNode(node_name, options)
  {
    init();
  }

  explicit SteeRepublisherNodeImu(
    const std::string & node_name, const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : SteeNode(node_name, namespace_, options)
  {
    init();
  }

  virtual ~SteeRepublisherNodeImu() {}

private:
  std::vector<std::string> original_input_topics_;
  std::string original_output_topic_;

  // subscriptions for original input topics
  std::vector<SteeSubscription<Imu>::SharedPtr> input_stee_subscriptions_;

  // subscription + publisher for republish
  rclcpp::Subscription<Imu>::SharedPtr output_subscription_;
  SteePublisher<Imu>::SharedPtr output_republisher_;

  void init()
  {
    original_input_topics_ =
      declare_parameter<std::vector<std::string>>("input_topics", std::vector<std::string>{});
    original_output_topic_ = declare_parameter<std::string>("output_topic", "");

    for (const auto & input_topic : original_input_topics_) {
      auto sub = create_stee_subscription<Imu>(
        input_topic,
        rclcpp::QoS(1).best_effort(),  // TODO(y-okumura-isp): parameterize QoS
        [](Imu::UniquePtr msg) { (void)msg; });
      input_stee_subscriptions_.push_back(sub);
    }

    output_republisher_ = create_stee_republisher<Imu>(original_output_topic_, rclcpp::QoS(1));
    output_subscription_ = create_subscription<Imu>(
      original_output_topic_, rclcpp::QoS(1).best_effort(), [this](Imu::UniquePtr msg) {
        output_republisher_->publish(std::move(msg));
      });  // TODO(y-okumura-isp): parameterize QoS
  }
};

}  // namespace tilde

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tilde::SteeRepublisherNodeImu)
