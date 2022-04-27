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

#include "tilde_message_filters/tilde_subscriber.hpp"

using namespace std::chrono_literals;

typedef sensor_msgs::msg::PointCloud2 Msg;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2 const> MsgConstPtr;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2> MsgPtr;

namespace sample_tilde_message_filter
{
struct NonConstHelper
{
  NonConstHelper(std::shared_ptr<tilde::TildePublisher<Msg>> pub):
      pub_(pub) {}

  void cb(const MsgPtr msg)
  {
    std::cout << "NonConstHelper::cb" << std::endl;
    pub_->publish(*msg);
  }

  MsgPtr msg_;
  std::shared_ptr<tilde::TildePublisher<Msg>> pub_;
};

std::shared_ptr<tilde::TildePublisher<Msg>> g_pub_callback_fn_;

void callback_fn(MsgConstPtr msg)
{
  std::cout << "callback_fn" << std::endl;
  g_pub_callback_fn_->publish(*msg);
}

template <typename CallbackT>
void func(CallbackT && callback)  // add [[deprecated]] to show deduced type
{
  auto callback_addr = &callback;
  std::cout << callback_addr << std::endl;
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

    sub_pc_.subscribe(this, "in1", qos.get_rmw_qos_profile());

    // lambda lvalue
    // registerCallback(const C& callback) with C = sample_tilde_message_filter::..::(lambda)...
    pub_lambda_lvalue_ = create_tilde_publisher<Msg>("out_lambda_lvalue", 1);
    auto sub_callback =
      [this](MsgConstPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "sub_callback");
        pub_lambda_lvalue_->publish(*msg); // rclcpp::Publisher::publish does not have shared_ptr version
      };
    sub_pc_.registerCallback(sub_callback);

    // lambda rvalue
    // const C& callback (be aware `C& callback` not defined)
    pub_lambda_rvalue_ = create_tilde_publisher<Msg>("out_lambda_rvalue", 1);
    sub_pc_.registerCallback(
        [this](MsgConstPtr msg) -> void
        {
          RCLCPP_INFO(this->get_logger(), "rvalue lambda");
          pub_lambda_rvalue_->publish(*msg);
        });

    /* type check
    std::cout << &sub_callback << std::endl;
    func(sub_callback);
    func([this](MsgConstPtr msg) -> void
        {
          (void) msg;
          RCLCPP_INFO(this->get_logger(), "rvalue lambda");
        });
    func([this](MsgConstPtr msg) -> void
        {
          (void) msg;
          RCLCPP_INFO(this->get_logger(), "rvalue lambda");
        });
    */

    // std::function
    // registerCallback(const std::function<void(P)>& callback)
    pub_callback2_ = create_tilde_publisher<Msg>("out_std_function_lvalue", 1);
    std::function<void(MsgConstPtr)> stdfunc = std::bind(&SampleSubscriberWithHeader::callback2, this, std::placeholders::_1);
    sub_pc_.registerCallback(stdfunc);

    /*
     * We can not use unique_ptr because message_event.h does not accept
     * registerCallback(const std::function<void(P)>& callback)
     */
    // std::function<void(std::unique_ptr<Msg>)> stdfunc_unique_ptr =
    //     std::bind(&SampleSubscriberWithHeader::callback_unique_ptr, this, std::placeholders::_1);
    // sub_pc_.registerCallback(stdfunc_unique_ptr);

    // std::bind rvalue
    // const C& callback
    sub_pc_.registerCallback(std::bind(&SampleSubscriberWithHeader::callback2, this, std::placeholders::_1));

    // std::bind lvalue but use auto
    // const C& callback.  => with C = std::_Bind<void ....>
    auto autofunc = std::bind(&SampleSubscriberWithHeader::callback2, this, std::placeholders::_1);
    sub_pc_.registerCallback(autofunc);

    // registerCallback(void(*callback)(P))
    // void(*callback)(P)
    g_pub_callback_fn_ = create_tilde_publisher<Msg>("out_callback_fn", 1);
    sub_pc_.registerCallback(callback_fn);

    // registerCallback(void(T::*callback)(P), T* t)
    pub_non_const_helper_ = create_tilde_publisher<Msg>("out_non_const_helper", 1);
    non_const_helper_ = std::make_shared<NonConstHelper>(pub_non_const_helper_);
    sub_pc_.registerCallback(&NonConstHelper::cb, non_const_helper_.get());
  }

private:
  tilde_message_filters::TildeSubscriber<Msg> sub_pc_;
  std::shared_ptr<tilde::TildePublisher<Msg>> pub_lambda_lvalue_;
  std::shared_ptr<tilde::TildePublisher<Msg>> pub_lambda_rvalue_;
  std::shared_ptr<tilde::TildePublisher<Msg>> pub_std_function_lvalue;
  std::shared_ptr<tilde::TildePublisher<Msg>> pub_callback2_;
  std::shared_ptr<tilde::TildePublisher<Msg>> pub_non_const_helper_;
  std::shared_ptr<NonConstHelper> non_const_helper_;

  void callback2(MsgConstPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "callback2");
    pub_callback2_->publish(*msg);
  }

  void callback_unique_ptr(std::unique_ptr<Msg> msg)
  {
    RCLCPP_INFO(this->get_logger(), "callback_unique_ptr");
    pub_callback2_->publish(std::move(msg));
  }

  MsgPtr msg_;
};

}  // namespace tilde_sample

RCLCPP_COMPONENTS_REGISTER_NODE(sample_tilde_message_filter::SampleSubscriberWithHeader)
