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
// #include <fstream>

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

using namespace std::chrono_literals;  // NOLINT

const int64_t TIMER_MS_DEFAULT = 1000;
const int64_t PROC_MS_DEFAULT = 10;

namespace tilde_sample
{
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class RelayTimerLoan : public tilde::TildeNode
{
public:
  explicit RelayTimerLoan(const rclcpp::NodeOptions & options)
  : TildeNode("relay_timer_loan", options)
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

    sub_loan_ = this->create_tilde_subscription<tilde_msg::msg::StaticSize>(
      "topic_without_stamp_loan", qos, [this](tilde_msg::msg::StaticSize::SharedPtr msg) -> void {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        auto now = tv.tv_sec * 1000 * 1000 + tv.tv_usec;
        uint64_t latency = now - msg->timestamp;

        RCLCPP_INFO(
          this->get_logger(), "(tilde_iceoryx-support) I heard message ID: '%ld', latency = %ld us",
          msg->id, latency);

        // write to .csv file(latency)
        // std::ofstream oFile;
        // oFile.open("tilde_iceoryx_support_latency.csv", std::ios::app);
        // oFile << msg->id << "," << latency << std::endl;
        // oFile.close();

        msg_loan_ = msg;
      });

    // Create a function for when messages are to be sent.
    auto proc_dur = std::chrono::duration<int64_t, std::milli>(proc_ms);
    auto timer_callback = [this, proc_dur, update_stamp]() -> void {
      std::this_thread::sleep_for(proc_dur);

      if (msg_loan_) {
        pub_->publish(*msg_loan_);
      }
    };

    // Create a publisher with a custom Quality of Service profile.
    pub_ =
      this->create_tilde_publisher<tilde_msg::msg::StaticSize>("relay_without_stamp_loan", qos);

    auto timer_dur = std::chrono::duration<int64_t, std::milli>(timer_ms);
    timer_ = this->create_wall_timer(timer_dur, timer_callback);
  }

private:
  std::shared_ptr<tilde_msg::msg::StaticSize> msg_loan_;
  rclcpp::Subscription<tilde_msg::msg::StaticSize>::SharedPtr sub_loan_;
  tilde::TildePublisher<tilde_msg::msg::StaticSize>::SharedPtr pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  uint64_t nanoseconds(const builtin_interfaces::msg::Time & time_msg)
  {
    rclcpp::Time time(time_msg);
    return time.nanoseconds();
  }
};

}  // namespace tilde_sample

RCLCPP_COMPONENTS_REGISTER_NODE(tilde_sample::RelayTimerLoan)
