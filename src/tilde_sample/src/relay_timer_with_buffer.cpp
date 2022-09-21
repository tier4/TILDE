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
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;  // NOLINT

const int64_t TIMER_MS_DEFAULT = 1000;
const int64_t PROC_MS_DEFAULT = 10;

namespace tilde_sample
{
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class RelayTimerWithBuffer : public tilde::TildeNode
{
public:
  explicit RelayTimerWithBuffer(const rclcpp::NodeOptions & options)
  : TildeNode("relay_timer_with_buffer", options)
  {
    const std::string TIMER_MS = "timer_ms";
    const std::string PROC_MS = "proc_ms";
    const std::string UPDATE_STAMP = "update_stamp";

    declare_parameter<int64_t>(TIMER_MS, TIMER_MS_DEFAULT);
    declare_parameter<int64_t>(PROC_MS, PROC_MS_DEFAULT);
    declare_parameter<bool>(UPDATE_STAMP, false);
    auto timer_ms = get_parameter(TIMER_MS).get_value<int64_t>();
    auto proc_ms = get_parameter(PROC_MS).get_value<int64_t>();
    auto update_stamp = get_parameter(UPDATE_STAMP).get_value<bool>();

    rclcpp::QoS qos(rclcpp::KeepLast(7));

    sub_pc_ = this->create_tilde_subscription<sensor_msgs::msg::PointCloud2>(
      "topic_with_stamp", qos, [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void {
        RCLCPP_INFO(
          this->get_logger(), "RelayTimer sub PointCloud2 seq: %s stamp: %lu",
          msg->header.frame_id.c_str(), nanoseconds(msg->header.stamp));
        msgs_pc_.push_back(std::move(msg));
      });

    sub_str_ = this->create_tilde_subscription<std_msgs::msg::String>(
      "topic_without_stamp", qos, [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "RelayTimer sub String seq: %s", msg->data.c_str());
        msgs_str_.push_back(std::move(msg));
      });

    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto proc_dur = std::chrono::duration<int64_t, std::milli>(proc_ms);
    auto timer_callback = [this, proc_dur, update_stamp]() -> void {
      std::this_thread::sleep_for(proc_dur);

      if (msgs_pc_.size() > 0) {
        auto msg = std::move(this->msgs_pc_.back());
        msgs_pc_.pop_back();

        // We call explicit API in this example unlike RelayTimer
        pub_pc_->add_explicit_input_info(this->sub_pc_->get_topic_name(), msg->header.stamp);

        auto stamp = msg->header.stamp;

        if (update_stamp) {
          stamp = this->now();
          msg->header.stamp = stamp;
        }

        RCLCPP_INFO(
          this->get_logger(), "RelayTimer pub PointCloud2 seq: %s stamp: %lu",
          msg->header.frame_id.c_str(), nanoseconds(msg->header.stamp));

        pub_pc_->publish(std::move(msg));
      }

      if (msgs_str_.size() > 0) {
        auto msg = std::move(this->msgs_str_.back());
        msgs_str_.pop_back();

        // We can not call explicit API because header.stamp does not exist

        RCLCPP_INFO(this->get_logger(), "RelayTimer pub String seq: %s", msg->data.c_str());

        pub_str_->publish(std::move(msg));
      }
    };

    // Create a publisher with a custom Quality of Service profile.
    pub_pc_ =
      this->create_tilde_publisher<sensor_msgs::msg::PointCloud2>("relay_buffer_with_stamp", qos);
    pub_str_ =
      this->create_tilde_publisher<std_msgs::msg::String>("relay_buffer_without_stamp", qos);

    // Use a timer to schedule periodic message publishing.
    auto timer_dur = std::chrono::duration<int64_t, std::milli>(timer_ms);
    timer_ = this->create_wall_timer(timer_dur, timer_callback);
  }

private:
  std::vector<std::unique_ptr<sensor_msgs::msg::PointCloud2>> msgs_pc_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
  tilde::TildePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_;

  std::vector<std::unique_ptr<std_msgs::msg::String>> msgs_str_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_str_;
  tilde::TildePublisher<std_msgs::msg::String>::SharedPtr pub_str_;

  rclcpp::TimerBase::SharedPtr timer_;

  uint64_t nanoseconds(const builtin_interfaces::msg::Time & time_msg)
  {
    rclcpp::Time time(time_msg);
    return time.nanoseconds();
  }
};

}  // namespace tilde_sample

RCLCPP_COMPONENTS_REGISTER_NODE(tilde_sample::RelayTimerWithBuffer)
