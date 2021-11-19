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
#include <iostream>
#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace pathnode_sample
{

const std::string REENTRANT = "reentrant";
const std::string SUB1_WAIT_MS = "sub1_wait_ms";
const std::string SUB2_WAIT_MS = "sub2_wait_ms";
const std::string SUB3_WAIT_MS = "sub3_wait_ms";
const std::string TIMER1_WAIT_MS = "timer1_wait_ms";
const std::string TIMER2_WAIT_MS = "timer2_wait_ms";
const std::string TIMER1_INTERVAL_MS = "timer1_interval_ms";
const std::string TIMER2_INTERVAL_MS = "timer2_interval_ms";

/// multiple callback for checking callback blocking
class SampleMultiCallback : public rclcpp::Node
{
public:
  explicit SampleMultiCallback(const rclcpp::NodeOptions & options)
  : Node("sample_multi_callback", options)
  {
    declare_parameter<bool>(REENTRANT, false);
    declare_parameter<int64_t>(SUB1_WAIT_MS, (int64_t) 0);
    declare_parameter<int64_t>(SUB2_WAIT_MS, (int64_t) 0);
    declare_parameter<int64_t>(SUB3_WAIT_MS, (int64_t) 0);
    declare_parameter<int64_t>(TIMER1_WAIT_MS, (int64_t) 0);
    declare_parameter<int64_t>(TIMER2_WAIT_MS, (int64_t) 0);
    declare_parameter<int64_t>(TIMER1_INTERVAL_MS, (int64_t) 1000);
    declare_parameter<int64_t>(TIMER2_INTERVAL_MS, (int64_t) 1000);


    if (get_parameter(REENTRANT).get_value<bool>()) {
      callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    }

    int64_t sub1_wait_ms = get_parameter(SUB1_WAIT_MS).get_value<int64_t>();
    int64_t sub2_wait_ms = get_parameter(SUB2_WAIT_MS).get_value<int64_t>();
    int64_t sub3_wait_ms = get_parameter(SUB3_WAIT_MS).get_value<int64_t>();
    int64_t timer1_wait_ms = get_parameter(TIMER1_WAIT_MS).get_value<int64_t>();
    int64_t timer2_wait_ms = get_parameter(TIMER2_WAIT_MS).get_value<int64_t>();
    int64_t timer1_interval_ms = get_parameter(TIMER1_INTERVAL_MS).get_value<int64_t>();
    int64_t timer2_interval_ms = get_parameter(TIMER2_INTERVAL_MS).get_value<int64_t>();

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = callback_group_;

    pub1_ = create_publisher<std_msgs::msg::String>("pub1", 1);

    auto sub1_callback =
      [this, sub1_wait_ms](const std_msgs::msg::String & msg) -> void
      {
        std::cout << get_name() << ":sub1_callback " << msg.data << std::endl;
        sleep("sub1_callback", sub1_wait_ms);
        pub1_->publish(msg);
      };
    sub1_ = create_subscription<std_msgs::msg::String>(
      "sub1", 3, sub1_callback, sub_options);

    pub2_ = create_publisher<std_msgs::msg::String>("pub2", 1);
    auto sub2_callback =
      [this, sub2_wait_ms](const std_msgs::msg::String & msg) -> void
      {
        std::cout << get_name() << "sub2_callback " << msg.data << std::endl;
        sleep("sub2_callback", sub2_wait_ms);
        pub2_->publish(msg);
      };
    sub2_ = create_subscription<std_msgs::msg::String>(
      "sub2", 3, sub2_callback, sub_options);

    pub3_ = create_publisher<std_msgs::msg::String>("pub3", 1);
    auto sub3_callback =
      [this, sub3_wait_ms](const std_msgs::msg::String & msg) -> void
      {
        std::cout << get_name() << "sub3_callback " << msg.data << std::endl;
        sleep("sub3_callback", sub3_wait_ms);
        pub3_->publish(msg);
      };
    sub3_ = create_subscription<std_msgs::msg::String>(
      "sub3", 3, sub3_callback, sub_options);

    timer1_pub_ = create_publisher<std_msgs::msg::String>("timer_pub1", 1);
    timer1_ = create_wall_timer(
      std::chrono::duration<int64_t, std::milli>(timer1_interval_ms),
      [this, timer1_wait_ms]() -> void
      {
        sleep("timer1_callback", timer1_wait_ms);
        std_msgs::msg::String msg;
        msg.data = "timer1_callback";
        timer1_pub_->publish(msg);
      },
      callback_group_);

    timer2_pub_ = create_publisher<std_msgs::msg::String>("timer_pub2", 1);
    timer2_ = create_wall_timer(
      std::chrono::duration<int64_t, std::milli>(timer2_interval_ms),
      [this, timer2_wait_ms]() -> void
      {
        sleep("timer2_callback", timer2_wait_ms);
        std_msgs::msg::String msg;
        msg.data = "timer2_callback";
        timer2_pub_->publish(msg);
      },
      callback_group_);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub2_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub3_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub3_;

  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timer1_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timer2_pub_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  void sleep(
    const std::string & name,
    int64_t ms)
  {
    std::cout << get_name() << ":" << name << " start sleep " << ms << std::endl;
    std::this_thread::sleep_for(std::chrono::duration<int64_t, std::milli>(ms));
    std::cout << get_name() << ":" << name << " wakeup and return" << std::endl;
  }
};


} // pathnode_sample

RCLCPP_COMPONENTS_REGISTER_NODE(pathnode_sample::SampleMultiCallback)
