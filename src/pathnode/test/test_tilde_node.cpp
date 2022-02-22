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

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "pathnode/tilde_node.hpp"
#include "pathnode/tilde_publisher.hpp"
#include "path_info_msg/msg/pub_info.hpp"

using pathnode::TildeNode;
using pathnode::InputInfo;
using path_info_msg::msg::PubInfo;

class TestTildeNode : public ::testing::Test
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

std::string str(const builtin_interfaces::msg::Time & time)
{
  std::string s = "";
  s += std::to_string(time.sec);
  s += ".";
  s += std::to_string(time.nanosec);
  return s;
}

TEST_F(TestTildeNode, simple_case) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  auto sensor_node = std::make_shared<rclcpp::Node>("sensorNode", options);
  auto main_node = std::make_shared<TildeNode>("pubNode", options);
  auto checker_node = std::make_shared<rclcpp::Node>("checkerNode", options);

  auto sensor_pub = sensor_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "in_topic", 1);
  auto clock_pub = sensor_node->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", 1);

  // apply "/clock"
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock.sec = 123;
  clock_msg.clock.nanosec = 456;

  clock_pub->publish(clock_msg);
  rclcpp::spin_some(sensor_node);
  rclcpp::spin_some(main_node);

  // prepare pub/sub
  auto main_pub = main_node->create_tilde_publisher<sensor_msgs::msg::PointCloud2>(
    "out_topic", 1);
  auto main_sub = main_node->create_tilde_subscription<sensor_msgs::msg::PointCloud2>(
    "in_topic", 1,
    [&main_pub](sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void
    {
      std::cout << "main_sub_callback" << std::endl;
      (void)msg;
      main_pub->publish(std::move(msg));
    });

  bool checker_sub_called = false;
  auto checker_sub = checker_node->create_subscription<path_info_msg::msg::PubInfo>(
    "out_topic/info/pub", 1,
    [&checker_sub_called, clock_msg](path_info_msg::msg::PubInfo::UniquePtr pub_info_msg) -> void
    {
      checker_sub_called = true;
      std::cout << "checker_sub_callback" << std::endl;
      std::cout << "pub_info_msg: \n" <<
        "pub_time: " << str(pub_info_msg->output_info.pub_time) << "\n" <<
        "pub_time_steady: " << str(pub_info_msg->output_info.pub_time_steady) << "\n" <<
        std::endl;
      EXPECT_EQ(
        pub_info_msg->output_info.pub_time,
        clock_msg.clock);
    });

  // do scenario
  auto sensor_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  sensor_msg->header.stamp = sensor_node->now();
  sensor_pub->publish(std::move(sensor_msg));

  rclcpp::spin_some(sensor_node);
  rclcpp::spin_some(main_node);
  rclcpp::spin_some(checker_node);
  EXPECT_TRUE(checker_sub_called);
}
