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

#include "tilde/stee_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

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

TEST_F(TestSteeNode, stee_publisher_unique_ptr) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  auto main_node = std::make_shared<SteeNode>("stee_node", options);
  auto stee_pub = main_node->create_stee_publisher<PointCloud2>("topic", 1);

  auto checker_node = std::make_shared<rclcpp::Node>("checker_node");
  bool received_main = false;
  auto main_sub = checker_node->create_subscription<PointCloud2>(
      "topic", 1,
      [&received_main](PointCloud2::UniquePtr msg) -> void
      {
        EXPECT_EQ(msg->header.frame_id, "unique");
        received_main = true;
      });
  bool received_converted = false;
  auto converted_sub = checker_node->create_subscription<SteePointCloud2>(
      "topic/stee", 1,
      [&received_converted](SteePointCloud2::UniquePtr msg) -> void
      {
        EXPECT_EQ(msg->body.header.frame_id, "unique");
        received_converted = true;
      });

  auto unique_ptr_msg = std::make_unique<PointCloud2>();
  unique_ptr_msg->header.frame_id = "unique";

  stee_pub->publish(std::move(unique_ptr_msg));

  rclcpp::spin_some(checker_node);

  EXPECT_TRUE(received_main);
  EXPECT_TRUE(received_converted);
}

TEST_F(TestSteeNode, stee_publisher_const_reference) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  auto main_node = std::make_shared<SteeNode>("stee_node", options);
  auto stee_pub = main_node->create_stee_publisher<PointCloud2>("topic", 1);

  auto checker_node = std::make_shared<rclcpp::Node>("checker_node");
  bool received_main = false;
  auto main_sub = checker_node->create_subscription<PointCloud2>(
      "topic", 1,
      [&received_main](const PointCloud2 & msg) -> void
      {
        EXPECT_EQ(msg.header.frame_id, "unique");
        received_main = true;
      });
  bool received_converted = false;
  auto converted_sub = checker_node->create_subscription<SteePointCloud2>(
      "topic/stee", 1,
      [&received_converted](const SteePointCloud2 & msg) -> void
      {
        EXPECT_EQ(msg.body.header.frame_id, "unique");
        received_converted = true;
      });

  PointCloud2 msg;
  msg.header.frame_id = "unique";

  stee_pub->publish(msg);

  rclcpp::spin_some(checker_node);

  EXPECT_TRUE(received_main);
  EXPECT_TRUE(received_converted);
}
