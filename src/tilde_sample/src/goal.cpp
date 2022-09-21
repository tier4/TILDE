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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tilde/tilde_node.hpp"
#include "tilde/tilde_publisher.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>

using namespace std::chrono_literals;  // NOLINT

namespace tilde_sample
{
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Goal : public tilde::TildeNode
{
public:
  explicit Goal(const rclcpp::NodeOptions & options) : TildeNode("talker", options)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto sub_callback = [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void {
      (void)msg;
      RCLCPP_INFO(this->get_logger(), "received");
    };
    sub_pc_ =
      this->create_tilde_subscription<sensor_msgs::msg::PointCloud2>("in", qos, sub_callback);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
};

}  // namespace tilde_sample

RCLCPP_COMPONENTS_REGISTER_NODE(tilde_sample::Goal)
