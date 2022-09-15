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

using namespace std::chrono_literals;  // NOLINT

const int64_t TIMER_MS_DEFAULT = 1000;

namespace tilde_sample
{

/**
 * This class sends messages with header.stamp field.
 * see msg.header.frame_id to get the sequence number.
 */
class SamplePublisherWithStamp : public tilde::TildeNode
{
public:
  explicit SamplePublisherWithStamp(const rclcpp::NodeOptions & options)
  : TildeNode("talker_with_stamp", options)
  {
    const std::string TIMER_MS = "timer_ms";

    declare_parameter<int64_t>(TIMER_MS, TIMER_MS_DEFAULT);
    auto timer_ms = get_parameter(TIMER_MS).get_value<int64_t>();
    std::cout << "timer_ms: " << timer_ms << std::endl;

    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto publish_message = [this]() -> void {
      auto time_now = this->now();

      msg_pc_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
      msg_pc_->header.stamp = time_now;
      msg_pc_->header.frame_id = std::to_string(count_);
      pub_pc_->publish(std::move(msg_pc_));

      RCLCPP_INFO(
        this->get_logger(), "Publishing PointCloud2: %ld stamp: %lu", count_,
        time_now.nanoseconds());

      count_++;
    };

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_pc_ = this->create_tilde_publisher<sensor_msgs::msg::PointCloud2>("topic_with_stamp", qos);

    // Use a timer to schedule periodic message publishing.
    auto dur = std::chrono::duration<int64_t, std::milli>(timer_ms);
    timer_ = this->create_wall_timer(dur, publish_message);
  }

private:
  size_t count_ = 0;
  // message with standard header
  std::unique_ptr<sensor_msgs::msg::PointCloud2> msg_pc_;
  tilde::TildePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_;

  rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * This class sends messages without header.stamp field.
 */
class SamplePublisherWithoutStamp : public tilde::TildeNode
{
public:
  explicit SamplePublisherWithoutStamp(const rclcpp::NodeOptions & options)
  : TildeNode("talker_without_stamp", options)
  {
    const std::string TIMER_MS = "timer_ms";

    declare_parameter<int64_t>(TIMER_MS, TIMER_MS_DEFAULT);
    auto timer_ms = get_parameter(TIMER_MS).get_value<int64_t>();
    std::cout << "timer_ms: " << timer_ms << std::endl;

    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto publish_message = [this]() -> void {
      auto time_now = this->now();

      msg_string_ = std::make_unique<std_msgs::msg::String>();
      msg_string_->data = std::to_string(count_);
      pub_string_->publish(std::move(msg_string_));

      RCLCPP_INFO(
        this->get_logger(), "Publishing String: '%ld' at '%lu'", count_, time_now.nanoseconds());

      count_++;
    };

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_string_ = this->create_tilde_publisher<std_msgs::msg::String>("topic_without_stamp", qos);

    // Use a timer to schedule periodic message publishing.
    auto dur = std::chrono::duration<int64_t, std::milli>(timer_ms);
    timer_ = this->create_wall_timer(dur, publish_message);
  }

private:
  size_t count_ = 0;

  // message without standard header, especially header.stamp
  std::unique_ptr<std_msgs::msg::String> msg_string_;
  tilde::TildePublisher<std_msgs::msg::String>::SharedPtr pub_string_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tilde_sample

RCLCPP_COMPONENTS_REGISTER_NODE(tilde_sample::SamplePublisherWithStamp)
RCLCPP_COMPONENTS_REGISTER_NODE(tilde_sample::SamplePublisherWithoutStamp)
