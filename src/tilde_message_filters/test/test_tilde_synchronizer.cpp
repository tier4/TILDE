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
#include "tilde_message_filters/tilde_synchronizer.h"

using namespace message_filters;
using namespace tilde_message_filters;

using Node = rclcpp::Node;
using TildeNode = tilde::TildeNode;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Ptr = std::shared_ptr<PointCloud2>;
using PointCloud2ConstPtr = std::shared_ptr<PointCloud2 const>;
using Clock = rosgraph_msgs::msg::Clock;

class TestSynchronizer : public ::testing::Test
{
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestSynchronizer, exact_policy_2msgs) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  auto pub_node = std::make_shared<Node>("pub_node", options);
  auto pub1 = pub_node->create_publisher<PointCloud2>("in1", 1);
  auto pub2 = pub_node->create_publisher<PointCloud2>("in2", 1);
  auto clock_pub = pub_node->create_publisher<Clock>("/clock", 1);

  auto sub_node = std::make_shared<TildeNode>("sub_node", options);
  Subscriber<PointCloud2> sub1, sub2;
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  sub1.subscribe(sub_node, "in1", qos.get_rmw_qos_profile());
  sub2.subscribe(sub_node, "in2", qos.get_rmw_qos_profile());

  using SyncPolicy = sync_policies::ExactTime<PointCloud2, PointCloud2>;
  using Sync = TildeSynchronizer<SyncPolicy>;
  auto sync = std::make_shared<Sync>(
      sub_node.get(),
      SyncPolicy(5),
      sub1,
      sub2);
  bool sync_callback_called = false;
  sync->registerCallback(
      [&sync_callback_called](const PointCloud2ConstPtr &msg1,
         const PointCloud2ConstPtr &msg2) -> void
      {
        (void) msg1;
        (void) msg2;
        sync_callback_called = true;
      });

  auto send_clock =
      [clock_pub](int32_t sec, uint32_t nsec) -> void
      {
        Clock clock_msg;
        clock_msg.clock.sec = sec;
        clock_msg.clock.nanosec = nsec;
        clock_pub->publish(clock_msg);
      };

  auto spin =
      [pub_node, sub_node]() -> void
      {
        rclcpp::spin_some(pub_node);
        rclcpp::spin_some(sub_node);
      };

  // apply "/clock"
  send_clock(123, 456);
  spin();

  // PointCloud2 message to publish
  PointCloud2 msg;
  msg.header.stamp = pub_node->now();

  // pub1 & sub
  msg.header.frame_id = 1;
  pub1->publish(msg);
  spin();

  // update clock
  send_clock(234, 567);
  spin();

  // pub2
  msg.header.frame_id = 2;
  pub2->publish(msg);
  spin();

  // update clock

  // varify
  EXPECT_TRUE(sync_callback_called);
}
