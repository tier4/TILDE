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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "tilde/tilde_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tilde_message_filters/tilde_subscriber.h"

using namespace std::chrono_literals;

typedef sensor_msgs::msg::PointCloud2 Msg;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2 const> MsgConstPtr;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2> MsgPtr;

namespace sample_tilde_message_filter
{
struct NonConstHelper
{
  void cb(const MsgPtr msg)
  {
    msg_ = msg;
    std::cout << "NonConstHelper::cb" << std::endl;
  }

  MsgPtr msg_;
};

void callback_fn(MsgConstPtr msg)
{
  (void) msg;
  std::cout << "callback_fn" << std::endl;
}

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class SampleSubscriberWithHeader : public tilde::TildeNode
{
public:
  explicit SampleSubscriberWithHeader(const rclcpp::NodeOptions & options)
  : TildeNode("sub_with_header", options)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(7));

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    sub_pc_.subscribe(this, "topic_with_stamp", qos.get_rmw_qos_profile());

    // lambda lvalue
    // registerCallback(const C& callback) with C = sample_tilde_message_filter::..::(lambda)...
    auto sub_callback =
      [this](MsgConstPtr msg) -> void
      {
        (void) msg;
        RCLCPP_INFO(this->get_logger(), "sub_callback");
      };
    sub_pc_.registerCallback(sub_callback);

    // lambda rvalue
    // const C& callback (be aware `C& callback` not defined)
    sub_pc_.registerCallback(
        [this](MsgConstPtr msg) -> void
        {
          (void) msg;
          RCLCPP_INFO(this->get_logger(), "rvalue lambda");
        });

    // std::function
    // registerCallback(const std::function<void(P)>& callback)
    std::function<void(MsgConstPtr)> stdfunc = std::bind(&SampleSubscriberWithHeader::callback2, this, std::placeholders::_1);
    sub_pc_.registerCallback(stdfunc);

    // std::bind rvalue
    // const C& callback
    sub_pc_.registerCallback(std::bind(&SampleSubscriberWithHeader::callback2, this, std::placeholders::_1));

    // std::bind lvalue but use auto
    // const C& callback.  => with C = std::_Bind<void ....>
    auto autofunc = std::bind(&SampleSubscriberWithHeader::callback2, this, std::placeholders::_1);
    sub_pc_.registerCallback(autofunc);

    // registerCallback(void(*callback)(P))
    // void(*callback)(P)
    sub_pc_.registerCallback(callback_fn);

    // registerCallback(void(T::*callback)(P), T* t)
    NonConstHelper h;
    sub_pc_.registerCallback(&NonConstHelper::cb, &h);
  }

private:
  tilde_message_filters::TildeSubscriber<Msg> sub_pc_;

  void callback2(MsgConstPtr msg)
  {
    (void) msg;
    RCLCPP_INFO(this->get_logger(), "callback2");
  }

  MsgPtr msg_;
};

}  // namespace tilde_sample

RCLCPP_COMPONENTS_REGISTER_NODE(sample_tilde_message_filter::SampleSubscriberWithHeader)
