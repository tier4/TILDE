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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tilde/tilde_node.hpp"
#include "tilde_msg/msg/pub_info.hpp"
#include "tilde_msg/msg/sub_topic_time_info.hpp"

#include "tilde_deadline_detector/tilde_deadline_detector_node.hpp"

namespace tilde_deadline_detector
{

class TestTildeDeadlineDetectorNode : public ::testing::Test
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

TEST_F(TestTildeDeadlineDetectorNode, get_pub_info_topics)
{
  auto tilde_node = tilde::TildeNode("tilde_node");
  auto pub = tilde_node.create_tilde_publisher<sensor_msgs::msg::PointCloud2>("topic", 1);
  auto pub2 = tilde_node.create_publisher<sensor_msgs::msg::PointCloud2>("topic2", 1);
  auto pub3 = tilde_node.create_publisher<sensor_msgs::msg::PointCloud2>("topic2/info/pub", 1);

  std::this_thread::sleep_for(std::chrono::seconds(3));

  auto det_node = TildeDeadlineDetectorNode("node");
  auto topics = det_node.get_pub_info_topics();

  EXPECT_EQ(topics.size(), 1u);
  EXPECT_EQ(*topics.begin(), "/topic/info/pub");
}

}  // namespace tilde_deadline_detector
