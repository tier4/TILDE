// Copyright 2022 Research Institute of Systems Planning, Inc.
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

#include "tilde_msg/msg/static_size.hpp"
#include "std_msgs/msg/string.hpp"

#include <sys/time.h>

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
 * This class sends loan messages without header.stamp field.
 */
class SamplePublisherWithoutStampLoan : public tilde::TildeNode
{
public:
  explicit SamplePublisherWithoutStampLoan(const rclcpp::NodeOptions & options)
  : TildeNode("talker_without_stamp_loan", options)
  {
    const std::string TIMER_MS = "timer_ms";

    declare_parameter<int64_t>(TIMER_MS, TIMER_MS_DEFAULT);
    auto timer_ms = get_parameter(TIMER_MS).get_value<int64_t>();
    std::cout << "timer_ms: " << timer_ms << std::endl;

    // Create a function for when messages are to be sent.
    auto publish_loan_message = [this]() -> void {
      auto time_now = this->now();

      auto msg_loan_ = pub_loan_->borrow_loaned_message();
      msg_loan_.get().id = count_++;

      RCLCPP_INFO(this->get_logger(), "(tilde_iceoryx-support) Publishing Message ID: '%ld'", msg_loan_.get().id);

      struct timeval tv;
      gettimeofday(&tv, NULL);
      msg_loan_.get().timestamp = tv.tv_sec * 1000 * 1000 + tv.tv_usec;

      pub_loan_->publish(std::move(msg_loan_));
    };

    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast(7));
    pub_loan_ = this->create_tilde_publisher<tilde_msg::msg::StaticSize>("topic_without_stamp_loan", qos);

    // Use a timer to schedule periodic message publishing.
    auto dur = std::chrono::duration<int64_t, std::milli>(timer_ms);
    timer_ = this->create_wall_timer(dur, publish_loan_message);
  }

private:
  size_t count_ = 0;

  tilde::TildePublisher<tilde_msg::msg::StaticSize>::SharedPtr pub_loan_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tilde_sample

RCLCPP_COMPONENTS_REGISTER_NODE(tilde_sample::SamplePublisherWithoutStampLoan)