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
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include "tilde/tilde_publisher.hpp"

using sensor_msgs::msg::PointCloud2;
using std_msgs::msg::String;
using tilde::Process;

class TestStampProcessor : public ::testing::Test
{
};

struct MsgWithTopLevelStamp
{
  builtin_interfaces::msg::Time stamp;
};

TEST_F(TestStampProcessor, pointer_with_header_stamp) {
  PointCloud2 msg;

  rclcpp::Time expect(1, 2, RCL_ROS_TIME);
  msg.header.stamp = expect;
  rclcpp::Time t(3, 4, RCL_ROS_TIME);
  auto stamp = Process<PointCloud2>::get_timestamp(t, &msg);

  EXPECT_EQ(stamp, expect);
}

TEST_F(TestStampProcessor, pointer_without_header_stamp) {
  String msg;
  rclcpp::Time t(3, 4, RCL_ROS_TIME);
  auto stamp = Process<String>::get_timestamp(t, &msg);

  EXPECT_EQ(stamp, t);
}

TEST_F(TestStampProcessor, const_pointer_with_header_stamp) {
  PointCloud2 _msg;
  rclcpp::Time expect(1, 2, RCL_ROS_TIME);
  _msg.header.stamp = expect;

  const PointCloud2 msg(_msg);
  rclcpp::Time t(3, 4, RCL_ROS_TIME);
  auto stamp = Process<PointCloud2>::get_timestamp_from_const(t, &msg);

  EXPECT_EQ(stamp, expect);
}

TEST_F(TestStampProcessor, const_pointer_without_header_stamp) {
  const String msg;
  rclcpp::Time t(3, 4, RCL_ROS_TIME);
  auto stamp = Process<String>::get_timestamp_from_const(t, &msg);

  EXPECT_EQ(stamp, t);
}

TEST_F(TestStampProcessor, pointer_with_top_level_stamp) {
  MsgWithTopLevelStamp msg;

  rclcpp::Time expect(1, 2, RCL_ROS_TIME);
  msg.stamp = expect;
  rclcpp::Time t(3, 4, RCL_ROS_TIME);
  auto stamp = Process<decltype(msg)>::get_timestamp(t, &msg);

  EXPECT_EQ(stamp, expect);
}

