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

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tilde/tilde_node.hpp"
#include "tilde/tilde_publisher.hpp"

using namespace std::chrono_literals;

/**
 * Sensor -> TildeNode ->
 *      PC2           String
 */
namespace tilde_vis_test
{

const char in_topic[] = "in";
const char out_topic[] = "out";

class Sensor : public rclcpp::Node
{
public:
  explicit Sensor(const rclcpp::NodeOptions & options)
  : Node("sensor", options)
  {
    const std::string TIMER_DUR = "timer_us";
    const int64_t TIMER_DUR_DEFAULT_NS = 100 * 1000;
    declare_parameter<int64_t>(TIMER_DUR, TIMER_DUR_DEFAULT_NS);

    auto timer_dur = get_parameter(TIMER_DUR).get_value<int64_t>();
    rclcpp::QoS qos(rclcpp::KeepLast(7));

    pub_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(in_topic, qos);

    auto timer_callback =
      [this]() -> void
      {
        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        msg->header.stamp = this->now();
        pub_pc_->publish(std::move(msg));
      };
    auto dur = std::chrono::duration<int64_t, std::micro>(timer_dur);
    timer_ = this->create_wall_timer(dur, timer_callback);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class Relay : public tilde::TildeNode
{
public:
  explicit Relay(const rclcpp::NodeOptions & options)
  : TildeNode("relay", options)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));

    pub_pc_ = this->create_tilde_publisher<sensor_msgs::msg::PointCloud2>(
      out_topic, qos);

    sub_pc_ = this->create_tilde_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic, qos,
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void
      {
        (void) msg;
        RCLCPP_INFO(
          this->get_logger(),
          "Relay get message cnt_ = %d",
          cnt_);
        pub_pc_->publish(std::move(msg));
        cnt_++;
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
  tilde::TildePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_;
  int cnt_{0};
};

class Filter : public tilde::TildeNode
{
public:
  explicit Filter(const rclcpp::NodeOptions & options)
  : TildeNode("filter", options)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));

    pub_str_ = this->create_tilde_publisher<std_msgs::msg::String>(
      out_topic, qos);

    sub_pc_ = this->create_tilde_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic, qos,
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void
      {
        (void) msg;
        RCLCPP_INFO(
          this->get_logger(),
          "Filter get message cnt_ = %d",
          cnt_);
        auto omsg = std::make_unique<std_msgs::msg::String>();
        omsg->data = "hello";
        pub_str_->publish(std::move(omsg));
        cnt_++;
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
  tilde::TildePublisher<std_msgs::msg::String>::SharedPtr pub_str_;
  int cnt_{0};
};


}  // namespace tilde_vis_test

RCLCPP_COMPONENTS_REGISTER_NODE(tilde_vis_test::Sensor)
RCLCPP_COMPONENTS_REGISTER_NODE(tilde_vis_test::Relay)
RCLCPP_COMPONENTS_REGISTER_NODE(tilde_vis_test::Filter)
