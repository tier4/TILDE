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

struct MsgWithHeaderAndTopLevelStamp
{
  std_msgs::msg::Header header;
  builtin_interfaces::msg::Time stamp;
};

TEST_F(TestStampProcessor, pointer_with_header_stamp) {
  PointCloud2 msg;

  rclcpp::Time expect(1, 2, RCL_ROS_TIME);
  msg.header.stamp = expect;
  auto stamp = Process<PointCloud2>::get_timestamp(&msg);

  EXPECT_TRUE(stamp ? true : false);
  EXPECT_EQ(*stamp, expect);
}

TEST_F(TestStampProcessor, pointer_without_header_stamp) {
  String msg;
  auto stamp = Process<String>::get_timestamp(&msg);

  EXPECT_FALSE((stamp ? true : false));
}

TEST_F(TestStampProcessor, const_pointer_with_header_stamp) {
  PointCloud2 _msg;
  rclcpp::Time expect(1, 2, RCL_ROS_TIME);
  _msg.header.stamp = expect;

  const PointCloud2 msg(_msg);
  auto stamp = Process<PointCloud2>::get_timestamp_from_const(&msg);

  EXPECT_TRUE((stamp ? true : false));
  EXPECT_EQ(*stamp, expect);
}

TEST_F(TestStampProcessor, const_pointer_without_header_stamp) {
  const String msg;
  auto stamp = Process<String>::get_timestamp_from_const(&msg);

  EXPECT_FALSE((stamp ? true : false));
}

TEST_F(TestStampProcessor, pointer_with_top_level_stamp) {
  MsgWithTopLevelStamp msg;

  rclcpp::Time expect(1, 2, RCL_ROS_TIME);
  msg.stamp = expect;
  auto stamp = Process<decltype(msg)>::get_timestamp(&msg);

  EXPECT_FALSE(tilde::HasHeader<decltype(msg)>::value);
  EXPECT_TRUE(tilde::HasStamp<decltype(msg)>::value);
  EXPECT_TRUE(tilde::HasStampWithoutHeader<decltype(msg)>::value);
  EXPECT_TRUE((stamp ? true : false));
  EXPECT_EQ(*stamp, expect);
}

TEST_F(TestStampProcessor, const_pointer_with_top_level_stamp) {
  MsgWithTopLevelStamp _msg;
  rclcpp::Time expect(1, 2, RCL_ROS_TIME);
  _msg.stamp = expect;

  const MsgWithTopLevelStamp msg(_msg);
  auto stamp = Process<decltype(msg)>::get_timestamp_from_const(&msg);

  EXPECT_FALSE(tilde::HasHeader<decltype(msg)>::value);
  EXPECT_TRUE(tilde::HasStamp<decltype(msg)>::value);
  EXPECT_TRUE(tilde::HasStampWithoutHeader<decltype(msg)>::value);
  EXPECT_TRUE((stamp ? true : false));
  EXPECT_EQ(*stamp, expect);
}

TEST_F(TestStampProcessor, pointer_with_header_and_top_level_stamp) {
  MsgWithHeaderAndTopLevelStamp msg;

  rclcpp::Time expect(1, 2, RCL_ROS_TIME);
  rclcpp::Time unexpected(3, 4, RCL_ROS_TIME);
  msg.header.stamp = expect;
  msg.stamp = unexpected;
  auto stamp = Process<decltype(msg)>::get_timestamp(&msg);

  EXPECT_TRUE(tilde::HasHeader<decltype(msg)>::value);
  EXPECT_TRUE(tilde::HasStamp<decltype(msg)>::value);
  EXPECT_FALSE(tilde::HasStampWithoutHeader<decltype(msg)>::value);
  EXPECT_TRUE((stamp ? true : false));
  EXPECT_EQ(*stamp, expect);
}

TEST_F(TestStampProcessor, const_pointer_with_header_and_top_level_stamp) {
  MsgWithHeaderAndTopLevelStamp _msg;

  rclcpp::Time expect(1, 2, RCL_ROS_TIME);
  rclcpp::Time unexpected(3, 4, RCL_ROS_TIME);
  _msg.header.stamp = expect;
  _msg.stamp = unexpected;

  const MsgWithHeaderAndTopLevelStamp msg(_msg);
  auto stamp = Process<decltype(msg)>::get_timestamp_from_const(&msg);

  EXPECT_TRUE(tilde::HasHeader<decltype(msg)>::value);
  EXPECT_TRUE(tilde::HasStamp<decltype(msg)>::value);
  EXPECT_FALSE(tilde::HasStampWithoutHeader<decltype(msg)>::value);
  EXPECT_TRUE((stamp ? true : false));
  EXPECT_EQ(*stamp, expect);
}
