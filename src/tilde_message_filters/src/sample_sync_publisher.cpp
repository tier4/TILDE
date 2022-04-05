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

#include <cassert>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

const int64_t TIMER_MS_DEFAULT = 1000;

namespace sample_tilde_message_filter
{

/**
 * This class sends messages with header.stamp field.
 * see msg.header.frame_id to get the sequence number.
 */
class SamplePublisherWithHeader : public rclcpp::Node
{
  using PublisherPtr = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

 public:
  explicit SamplePublisherWithHeader(const rclcpp::NodeOptions & options)
  : Node("talker_with_header", options)
  {
    const std::string TIMER_MS = "timer_ms";
    const std::string N_PUBLISHERS = "n_publishers";

    declare_parameter<int64_t>(TIMER_MS, TIMER_MS_DEFAULT);
    auto timer_ms = get_parameter(TIMER_MS).get_value<int64_t>();
    std::cout << "timer_ms: " << timer_ms << std::endl;

    declare_parameter<int64_t>(N_PUBLISHERS, n_publishers_);
    n_publishers_ = get_parameter(N_PUBLISHERS).get_value<int64_t>();
    std::cout << "n_publishers: " << n_publishers_ << std::endl;
    assert(n_publishers_ >= 2);

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    for(auto i=0; i<n_publishers_; i++) {
      std::string topic = "in";
      topic += std::to_string(i);
      pub_pcs_.push_back(this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos));
    }

    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto publish_message =
      [this]() -> void
      {
        auto time_now = this->now();

        sensor_msgs::msg::PointCloud2 msg_pc;
        msg_pc.header.stamp = time_now;
        msg_pc.header.frame_id = std::to_string(count_);

        for(const auto &pub: pub_pcs_) {
          pub->publish(msg_pc);
        }

        RCLCPP_INFO(
          this->get_logger(), "Publishing PointCloud2: %ld stamp: %lu",
          count_,
          time_now.nanoseconds());

        count_++;
      };

    // Use a timer to schedule periodic message publishing.
    auto dur = std::chrono::duration<int64_t, std::milli>(timer_ms);
    timer_ = this->create_wall_timer(dur, publish_message);
  }

private:
  size_t count_ = 0;
  int64_t n_publishers_ = 9;
  std::vector<PublisherPtr> pub_pcs_;

  rclcpp::TimerBase::SharedPtr timer_;
};

/**
 * This class sends messages without header.stamp field.
 */
class SamplePublisherWithoutHeader : public rclcpp::Node
{
public:
  explicit SamplePublisherWithoutHeader(const rclcpp::NodeOptions & options)
  : Node("talker_without_header", options)
  {
    const std::string TIMER_MS = "timer_ms";

    declare_parameter<int64_t>(TIMER_MS, TIMER_MS_DEFAULT);
    auto timer_ms = get_parameter(TIMER_MS).get_value<int64_t>();
    std::cout << "timer_ms: " << timer_ms << std::endl;

    // Create a function for when messages are to be sent.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto publish_message =
      [this]() -> void
      {
        auto time_now = this->now();

        msg_string_ = std::make_unique<std_msgs::msg::String>();
        msg_string_->data = std::to_string(count_);
        pub_string_->publish(std::move(msg_string_));

        RCLCPP_INFO(
          this->get_logger(), "Publishing String: '%ld' at '%lu'",
          count_,
          time_now.nanoseconds());

        count_++;
      };

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_string_ = this->create_publisher<std_msgs::msg::String>("topic_without_header", qos);

    // Use a timer to schedule periodic message publishing.
    auto dur = std::chrono::duration<int64_t, std::milli>(timer_ms);
    timer_ = this->create_wall_timer(dur, publish_message);
  }

private:
  size_t count_ = 0;

  // message without standard header, especially header.stamp
  std::unique_ptr<std_msgs::msg::String> msg_string_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_string_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sample_tilde_message_filter

RCLCPP_COMPONENTS_REGISTER_NODE(sample_tilde_message_filter::SamplePublisherWithHeader)
RCLCPP_COMPONENTS_REGISTER_NODE(sample_tilde_message_filter::SamplePublisherWithoutHeader)
