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

#include <gtest/gtest.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "tilde/stee_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tilde_msg/msg/stee_point_cloud2.hpp"

using namespace std::chrono_literals;
using rclcpp::Node;
using tilde::SteeNode;
using sensor_msgs::msg::PointCloud2;
using tilde_msg::msg::SteePointCloud2;

class TestSteeNode : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

#if 0
TEST_F(TestSteeNode, stee_multi_thread) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", false);

  // sensor node
  auto sensor_node = std::make_shared<SteeNode>("sensor_node", options);
  auto sensor_pub1 = sensor_node->create_stee_publisher<PointCloud2>("topic1", 1);
  auto sensor_pub2 = sensor_node->create_stee_publisher<PointCloud2>("topic2", 1);

  // main node
  auto main_node = std::make_shared<SteeNode>("stee_node", options);
  auto main_pub = main_node->create_stee_publisher<PointCloud2>("main", 1);

  auto checker_node = std::make_shared<rclcpp::Node>("checker_node");

  auto cb1 = main_node->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  auto cb2 = main_node->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

  auto sub_opt1 = rclcpp::SubscriptionOptions();
  sub_opt1.callback_group = cb1;

  auto sub_opt2 = rclcpp::SubscriptionOptions();
  sub_opt2.callback_group = cb2;

  auto main_sub1 = main_node->create_stee_subscription<PointCloud2>(
    "topic1", 1,
    [&main_pub](PointCloud2::UniquePtr msg) -> void
    {
      std::cout << "topic1" << std::endl;
      main_pub->publish(std::move(msg));
    },
    sub_opt1);

  auto main_sub2 = main_node->create_stee_subscription<PointCloud2>(
    "topic2", 1,
    [&main_pub](PointCloud2::UniquePtr msg) -> void
    {
      std::cout << "topic2" << std::endl;
      main_pub->publish(std::move(msg));
    },
    sub_opt2);

  std::thread sender_thread(
      [&sensor_pub1, &sensor_pub2]() -> void
      {
        for (int i=0; i<10; i++) {
          std::cout << "publish " << std::to_string(i) << std::endl;
          PointCloud2 msg;
          msg.header.stamp.sec = i;
          msg.header.frame_id = std::to_string(i);
          sensor_pub1->publish(msg);
          sensor_pub2->publish(msg);
          std::this_thread::sleep_for(500ms);
        }
      });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(main_node);
  std::thread main_node_thread(
      [&executor]() -> void
      {
        executor.spin();
      });
  sender_thread.join();
  executor.cancel();
  main_node_thread.join();
}

// use rclcpp::Node case, for comparison
TEST_F(TestSteeNode, non_stee_multi_thread) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", false);

  // sensor node
  auto sensor_node = std::make_shared<Node>("sensor_node", options);
  auto sensor_pub1 = sensor_node->create_publisher<PointCloud2>("topic1", 1);
  auto sensor_pub2 = sensor_node->create_publisher<PointCloud2>("topic2", 1);

  // main node
  auto main_node = std::make_shared<Node>("stee_node", options);
  auto main_pub = main_node->create_publisher<PointCloud2>("main", 1);

  auto checker_node = std::make_shared<rclcpp::Node>("checker_node");

  auto cb1 = main_node->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  auto cb2 = main_node->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

  auto sub_opt1 = rclcpp::SubscriptionOptions();
  sub_opt1.callback_group = cb1;

  auto sub_opt2 = rclcpp::SubscriptionOptions();
  sub_opt2.callback_group = cb2;

  auto main_sub1 = main_node->create_subscription<PointCloud2>(
    "topic1", 1,
    [&main_pub](PointCloud2::UniquePtr msg) -> void
    {
      std::cout << "topic1" << std::endl;
      main_pub->publish(std::move(msg));
    },
    sub_opt1);

  auto main_sub2 = main_node->create_subscription<PointCloud2>(
    "topic2", 1,
    [&main_pub](PointCloud2::UniquePtr msg) -> void
    {
      std::cout << "topic2" << std::endl;
      main_pub->publish(std::move(msg));
    },
    sub_opt2);

  std::thread sender_thread(
      [&sensor_pub1, &sensor_pub2]() -> void
      {
        for (int i=0; i<10; i++) {
          std::cout << "publish " << std::to_string(i) << std::endl;
          PointCloud2 msg;
          msg.header.stamp.sec = i;
          msg.header.frame_id = std::to_string(i);
          sensor_pub1->publish(msg);
          sensor_pub2->publish(msg);
          std::this_thread::sleep_for(500ms);
        }
      });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(main_node);
  std::thread main_node_thread(
      [&executor]() -> void
      {
        executor.spin();
      });
  sender_thread.join();
  executor.cancel();
  main_node_thread.join();
}
#endif

/// test for explicit API
TEST_F(TestSteeNode, stee_multi_thread_with_explicit_api) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", false);

  // sensor node
  auto sensor_node = std::make_shared<SteeNode>("sensor_node", options);
  auto sensor_pub1 = sensor_node->create_stee_publisher<PointCloud2>("topic1", 1);
  auto sensor_pub2 = sensor_node->create_stee_publisher<PointCloud2>("topic2", 1);

  // main node
  auto main_node = std::make_shared<SteeNode>("stee_node", options);
  auto main_pub = main_node->create_stee_publisher<PointCloud2>("main", 1);

  auto checker_node = std::make_shared<rclcpp::Node>("checker_node");

  auto cb1 = main_node->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  auto cb2 = main_node->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

  auto sub_opt1 = rclcpp::SubscriptionOptions();
  sub_opt1.callback_group = cb1;

  auto sub_opt2 = rclcpp::SubscriptionOptions();
  sub_opt2.callback_group = cb2;

  auto main_sub1 = main_node->create_stee_subscription<PointCloud2>(
    "topic1", 1,
    [&main_pub](PointCloud2::UniquePtr msg) -> void
    {
      std::cout << "topic1" << std::endl;
      main_pub->add_explicit_input_info(
          "/topic1",
          msg->header.stamp);
      main_pub->publish(std::move(msg));
    },
    sub_opt1);

  auto main_sub2 = main_node->create_stee_subscription<PointCloud2>(
    "/topic2", 1,
    [&main_pub](PointCloud2::UniquePtr msg) -> void
    {
      std::cout << "topic2" << std::endl;
      main_pub->add_explicit_input_info(
          "topic2",
          msg->header.stamp);
      main_pub->publish(std::move(msg));
    },
    sub_opt2);

  std::thread sender_thread(
      [&sensor_pub1, &sensor_pub2]() -> void
      {
        for (int i=0; i<10; i++) {
          std::cout << "publish " << std::to_string(i) << std::endl;
          PointCloud2 msg;
          msg.header.stamp.sec = i;
          msg.header.frame_id = std::to_string(i);
          sensor_pub1->publish(msg);
          sensor_pub2->publish(msg);
          std::this_thread::sleep_for(500ms);
        }
      });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(main_node);
  std::thread main_node_thread(
      [&executor]() -> void
      {
        executor.spin();
      });
  sender_thread.join();
  executor.cancel();
  main_node_thread.join();
}
