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

#include "tilde_message_filters/tilde_subscriber.hpp"

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

    options.append_parameter_override("use_sim_time", true);
    qos.keep_last(10);

    // setup pub node
    pub_node = std::make_shared<Node>("pub_node", options);
    for(auto i=0u; i<pubs.size(); i++) {
      auto topic = std::string("in") + std::to_string(i + 1);
      pubs[i] = pub_node->create_publisher<PointCloud2>(topic, qos);
    }
    clock_pub = pub_node->create_publisher<Clock>("/clock", qos);

    // setup sub node
    sub_node = std::make_shared<TildeNode>("sub_node", options);
    // skip init subs. Get subs via init_and_get_sub().
    out_pub = sub_node->create_tilde_publisher<PointCloud2>("out", qos);

    // setup val node
    val_node = std::make_shared<Node>("val_node", options);
  }

    void TearDown() override
  {
    rclcpp::shutdown();
  }

  Clock send_clock(int32_t sec, uint32_t nsec)
  {
    Clock clock_msg;
    clock_msg.clock.sec = sec;
    clock_msg.clock.nanosec = nsec;
    clock_pub->publish(clock_msg);
    return clock_msg;
  };

  Clock send_clock_and_spin(int32_t sec, uint32_t nsec)
  {
    auto clock = send_clock(sec, nsec);
    spin();
    return clock;
  };

  void spin()
  {
    rclcpp::spin_some(pub_node);
    rclcpp::spin_some(sub_node);
    rclcpp::spin_some(val_node);
  }

  rclcpp::NodeOptions options;
  rclcpp::QoS qos{10};

  std::shared_ptr<Node> pub_node;
  PointCloudPublishers pubs{9};
  std::shared_ptr<rclcpp::Publisher<Clock>> clock_pub;

  std::shared_ptr<TildeNode> sub_node;
  PointCloudTildeSubscriber sub;
  std::shared_ptr<TildePublisher<PointCloud2>> out_pub;

  std::shared_ptr<Node> val_node;
};

void EXPECT_CLOCK(rclcpp::Time time, int sec, uint32_t nsec)
{
  TimeMsg t = time;
  EXPECT_EQ(t.sec, sec);
  EXPECT_EQ(t.nanosec, nsec);
}

TEST_F(TestTildeSubscriber, hold_subtime)
{
  auto &pub1 = pubs[0];
  sub.subscribe(sub_node, "in1", qos.get_rmw_qos_profile());

  // time
  const int32_t t1_sec{1}, t2_sec{1}, t3_sec{2};
  const uint32_t t1_nsec{50}, t2_nsec{100}, t3_nsec{150};

  // PointCloud2 message to publish
  PointCloud2 msg;

  // t=t1
  send_clock(t1_sec, t1_nsec);
  spin();
  EXPECT_CLOCK(sub_node->now(), t1_sec, t1_nsec);

  // in1: stamp=t2 @ t1
  msg.header.stamp.sec = t2_sec;
  msg.header.stamp.nanosec = t2_nsec;
  msg.header.frame_id = 1;
  pub1->publish(msg);
  spin();

  // t=t3
  send_clock(t3_sec, t3_nsec);
  spin();
  EXPECT_CLOCK(sub_node->now(), t3_sec, t3_nsec);

  // check
  rclcpp::Time subtime, subtime_steady;
  sub_node->find_subtime(
      &msg,
      "/in1",
      subtime,
      subtime_steady);
  (void) subtime_steady;
  EXPECT_CLOCK(subtime, t1_sec, t1_nsec);
}

