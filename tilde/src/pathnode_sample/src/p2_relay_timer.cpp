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
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pathnode/sub_timing_advertise_node.hpp"
#include "pathnode/timing_advertise_publisher.hpp"

using namespace std::chrono_literals;

namespace pathnode_sample
{
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class P2RelayTimer : public pathnode::SubTimingAdvertiseNode
{
public:
  explicit P2RelayTimer(const rclcpp::NodeOptions & options)
  : SubTimingAdvertiseNode("talker", options)
  {
    const std::string TIMER_MS = "timer_ms";
    const std::string PROC_MS = "proc_ms";
    const std::string UPDATE_STAMP = "update_stamp";

    declare_parameter<int64_t>(TIMER_MS, (int64_t) 10);
    declare_parameter<int64_t>(PROC_MS, (int64_t) 20);
    declare_parameter<bool>(UPDATE_STAMP, false);
    auto timer_ms = get_parameter(TIMER_MS).get_value<int64_t>();
    auto proc_ms = get_parameter(PROC_MS).get_value<int64_t>();
    auto update_stamp = get_parameter(UPDATE_STAMP).get_value<bool>();

    rclcpp::QoS qos(rclcpp::KeepLast(7));

    auto sub_callback =
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "RelayTimer sub callback");
        msg_pc_ = std::move(msg);
      };
    sub_pc_ = this->create_timing_advertise_subscription<sensor_msgs::msg::PointCloud2>(
      "in", qos,
      sub_callback);

    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto proc_dur = std::chrono::duration<int64_t, std::milli>(proc_ms);
    auto timer_callback =
      [this, proc_dur, update_stamp]() -> void
      {
        if (msg_pc_ == nullptr) {return;}

        // TODO(y-okumura-isp) wait
        std::this_thread::sleep_for(proc_dur);

        RCLCPP_INFO(this->get_logger(), "RelayTimer publish");
        msg_pc_ = std::make_unique<sensor_msgs::msg::PointCloud2>();

        if (update_stamp) {
          msg_pc_->header.stamp = this->now();
        }
        pub_pc_->publish(std::move(msg_pc_));
      };

    // Create a publisher with a custom Quality of Service profile.
    pub_pc_ = this->create_timing_advertise_publisher<sensor_msgs::msg::PointCloud2>("out", qos);

    // Use a timer to schedule periodic message publishing.
    auto timer_dur = std::chrono::duration<int64_t, std::milli>(timer_ms);
    timer_ = this->create_wall_timer(timer_dur, timer_callback);
  }

private:
  std::unique_ptr<sensor_msgs::msg::PointCloud2> msg_pc_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
  pathnode::TimingAdvertisePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pathnode_sample

RCLCPP_COMPONENTS_REGISTER_NODE(pathnode_sample::P2RelayTimer)
