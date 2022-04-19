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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "tilde_message_filters/tilde_subscriber.h"

using namespace tilde;
using namespace message_filters;
using namespace tilde_message_filters;

using Node = rclcpp::Node;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Ptr = std::shared_ptr<PointCloud2>;
using PointCloud2ConstPtr = std::shared_ptr<PointCloud2 const>;

using Clock = rosgraph_msgs::msg::Clock;
using PubInfo = tilde_msg::msg::PubInfo;
using PubInfoPtr = PubInfo::UniquePtr;
using TimeMsg = builtin_interfaces::msg::Time;

using PointCloudPublisher = std::shared_ptr<rclcpp::Publisher<PointCloud2>>;
using PointCloudPublishers = std::vector<PointCloudPublisher>;
using PointCloudTildeSubscriber = TildeSubscriber<PointCloud2>;
using PointCloudTildeSubscribers = std::vector<PointCloudTildeSubscriber>;

class TestTildeSubscriber : public ::testing::Test
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

TEST_F(TestTildeSubscriber, hoge)
{
  EXPECT_TRUE(true);
}

