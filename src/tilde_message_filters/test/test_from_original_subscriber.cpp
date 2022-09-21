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
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <gtest/gtest.h>

// see ros2/rclcpp#1619,1713
// TODO: remove this comment, and the `NonConstHelper` tests // NOLINT
// once the deprecated signatures have been discontinued.
#define RCLCPP_AVOID_DEPRECATIONS_FOR_UNIT_TESTS 1
#include "message_filters/chain.h"
#include "tilde/tilde_node.hpp"
#include "tilde_message_filters/tilde_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "sensor_msgs/msg/imu.hpp"

using namespace message_filters;        // NOLINT
using namespace tilde_message_filters;  // NOLINT
typedef sensor_msgs::msg::Imu Msg;
typedef std::shared_ptr<sensor_msgs::msg::Imu const> MsgConstPtr;
typedef std::shared_ptr<sensor_msgs::msg::Imu> MsgPtr;

class Helper
{
public:
  Helper() : count_(0) {}

  void cb(const MsgConstPtr) { ++count_; }

  int32_t count_;
};

TEST(TildeSubscriber, simple)
{
  auto node = std::make_shared<tilde::TildeNode>("test_node");
  Helper h;
  TildeSubscriber<Msg> sub(node, "test_topic");
  sub.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));
  auto pub = node->create_publisher<Msg>("test_topic", 10);
  rclcpp::Clock ros_clock;
  auto start = ros_clock.now();
  while (h.count_ == 0 && (ros_clock.now() - start) < rclcpp::Duration(1, 0)) {
    pub->publish(Msg());
    rclcpp::Rate(50).sleep();
    rclcpp::spin_some(node);
  }

  ASSERT_GT(h.count_, 0);
}

TEST(TildeSubscriber, simple_raw)
{
  auto node = std::make_shared<tilde::TildeNode>("test_node");
  Helper h;
  TildeSubscriber<Msg> sub(node.get(), "test_topic");
  sub.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));
  auto pub = node->create_publisher<Msg>("test_topic", 10);
  rclcpp::Clock ros_clock;
  auto start = ros_clock.now();
  while (h.count_ == 0 && (ros_clock.now() - start) < rclcpp::Duration(1, 0)) {
    pub->publish(Msg());
    rclcpp::Rate(50).sleep();
    rclcpp::spin_some(node);
  }

  ASSERT_GT(h.count_, 0);
}

TEST(TildeSubscriber, sub_UnSub_Sub)
{
  auto node = std::make_shared<tilde::TildeNode>("test_node");
  Helper h;
  TildeSubscriber<Msg> sub(node, "test_topic");
  sub.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));
  auto pub = node->create_publisher<Msg>("test_topic", 10);

  sub.unsubscribe();
  sub.subscribe();

  rclcpp::Clock ros_clock;
  auto start = ros_clock.now();
  while (h.count_ == 0 && (ros_clock.now() - start) < rclcpp::Duration(1, 0)) {
    pub->publish(Msg());
    rclcpp::Rate(50).sleep();
    rclcpp::spin_some(node);
  }

  ASSERT_GT(h.count_, 0);
}

TEST(TildeSubscriber, sub_UnSub_Sub_raw)
{
  auto node = std::make_shared<tilde::TildeNode>("test_node");
  Helper h;
  TildeSubscriber<Msg> sub(node.get(), "test_topic");
  sub.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));
  auto pub = node->create_publisher<Msg>("test_topic", 10);

  sub.unsubscribe();
  sub.subscribe();

  rclcpp::Clock ros_clock;
  auto start = ros_clock.now();
  while (h.count_ == 0 && (ros_clock.now() - start) < rclcpp::Duration(1, 0)) {
    pub->publish(Msg());
    rclcpp::Rate(50).sleep();
    rclcpp::spin_some(node);
  }

  ASSERT_GT(h.count_, 0);
}

TEST(TildeSubscriber, switchRawAndShared)
{
  auto node = std::make_shared<tilde::TildeNode>("test_node");
  Helper h;
  TildeSubscriber<Msg> sub(node, "test_topic");
  sub.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));
  auto pub = node->create_publisher<Msg>("test_topic2", 10);

  sub.unsubscribe();
  sub.subscribe(node.get(), "test_topic2");

  rclcpp::Clock ros_clock;
  auto start = ros_clock.now();
  while (h.count_ == 0 && (ros_clock.now() - start) < rclcpp::Duration(1, 0)) {
    pub->publish(Msg());
    rclcpp::Rate(50).sleep();
    rclcpp::spin_some(node);
  }

  ASSERT_GT(h.count_, 0);
}

TEST(TildeSubscriber, subInChain)
{
  auto node = std::make_shared<tilde::TildeNode>("test_node");
  Helper h;
  Chain<Msg> c;
  c.addFilter(std::make_shared<TildeSubscriber<Msg> >(node, "test_topic"));
  c.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));
  auto pub = node->create_publisher<Msg>("test_topic", 10);

  rclcpp::Clock ros_clock;
  auto start = ros_clock.now();
  while (h.count_ == 0 && (ros_clock.now() - start) < rclcpp::Duration(1, 0)) {
    pub->publish(Msg());
    rclcpp::Rate(50).sleep();
    rclcpp::spin_some(node);
  }

  ASSERT_GT(h.count_, 0);
}

struct ConstHelper
{
  void cb(const MsgConstPtr msg) { msg_ = msg; }

  MsgConstPtr msg_;
};

struct NonConstHelper
{
  void cb(const MsgPtr msg) { msg_ = msg; }

  MsgPtr msg_;
};

TEST(TildeSubscriber, singleNonConstCallback)
{
  auto node = std::make_shared<tilde::TildeNode>("test_node");
  NonConstHelper h;
  TildeSubscriber<Msg> sub(node, "test_topic");
  sub.registerCallback(&NonConstHelper::cb, &h);
  auto pub = node->create_publisher<Msg>("test_topic", 10);
  Msg msg;
  pub->publish(Msg());

  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(node);

  ASSERT_TRUE(h.msg_);
  ASSERT_EQ(msg, *h.msg_.get());
}

TEST(TildeSubscriber, multipleNonConstCallbacksFilterTildeSubscriber)
{
  auto node = std::make_shared<tilde::TildeNode>("test_node");
  NonConstHelper h, h2;
  TildeSubscriber<Msg> sub(node, "test_topic");
  sub.registerCallback(&NonConstHelper::cb, &h);
  sub.registerCallback(&NonConstHelper::cb, &h2);
  auto pub = node->create_publisher<Msg>("test_topic", 10);
  auto msg = std::make_unique<Msg>();
  pub->publish(std::move(msg));

  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(node);

  ASSERT_TRUE(h.msg_);
  ASSERT_TRUE(h2.msg_);
  EXPECT_NE(msg.get(), h.msg_.get());
  EXPECT_NE(msg.get(), h2.msg_.get());
  EXPECT_NE(h.msg_.get(), h2.msg_.get());
}

TEST(TildeSubscriber, multipleCallbacksSomeFilterSomeDirect)
{
  auto node = std::make_shared<tilde::TildeNode>("test_node");
  NonConstHelper h, h2;
  TildeSubscriber<Msg> sub(node, "test_topic");
  sub.registerCallback(&NonConstHelper::cb, &h);
  auto sub2 = node->create_subscription<Msg>(
    "test_topic", 10, std::bind(&NonConstHelper::cb, &h2, std::placeholders::_1));

  auto pub = node->create_publisher<Msg>("test_topic", 10);
  auto msg = std::make_unique<Msg>();
  pub->publish(std::move(msg));

  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(node);

  ASSERT_TRUE(h.msg_);
  ASSERT_TRUE(h2.msg_);
  EXPECT_NE(msg.get(), h.msg_.get());
  EXPECT_NE(msg.get(), h2.msg_.get());
  EXPECT_NE(h.msg_.get(), h2.msg_.get());
}

/*
TEST(TildeSubscriber, lifecycle)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  Helper h;
  TildeSubscriber<Msg, rclcpp_lifecycle::LifecycleNode> sub(node, "test_topic");
  sub.registerCallback(std::bind(&Helper::cb, &h, std::placeholders::_1));
  auto pub = node->create_publisher<Msg>("test_topic", 10);
  pub->on_activate();
  rclcpp::Clock ros_clock;
  auto start = ros_clock.now();
  while (h.count_ == 0 && (ros_clock.now() - start) < rclcpp::Duration(1, 0))
  {
    pub->publish(Msg());
    rclcpp::Rate(50).sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  ASSERT_GT(h.count_, 0);
}
*/

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
