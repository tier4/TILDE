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

class TestSynchronizer : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    options.append_parameter_override("use_sim_time", true);
    qos.keep_last(10);

    // setup pub node
    pub_node = std::make_shared<Node>("pub_node", options);
    for(auto i=0u; i<subs.size(); i++) {
      auto topic = std::string("in") + std::to_string(i + 1);
      pubs.push_back(pub_node->create_publisher<PointCloud2>(topic, qos));
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

  template<std::size_t I>
  PointCloudTildeSubscriber & init_and_get_sub()
  {
    static_assert(I < 9);
    auto topic = std::string("in") + std::to_string(I + 1);
    subs[I].subscribe(sub_node, topic, qos.get_rmw_qos_profile());
    return subs[I];
  }

  rclcpp::NodeOptions options;
  rclcpp::QoS qos{10};

  std::shared_ptr<Node> pub_node;
  PointCloudPublishers pubs;
  std::shared_ptr<rclcpp::Publisher<Clock>> clock_pub;

  std::shared_ptr<TildeNode> sub_node;
  PointCloudTildeSubscribers subs{9};
  std::shared_ptr<TildePublisher<PointCloud2>> out_pub;

  std::shared_ptr<Node> val_node;
};

void EXPECT_CLOCK(rclcpp::Time time, int sec, uint32_t nsec)
{
  TimeMsg t = time;
  EXPECT_EQ(t.sec, sec);
  EXPECT_EQ(t.nanosec, nsec);
}

TEST_F(TestSynchronizer, exact_policy_2msgs) {
  auto pub1 = pubs[0];
  auto pub2 = pubs[1];
  auto& sub1 = init_and_get_sub<0>();
  auto& sub2 = init_and_get_sub<1>();

  // setup sub_node synchronizer
  using SyncPolicy = sync_policies::ExactTime<PointCloud2, PointCloud2>;
  using Sync = TildeSynchronizer<SyncPolicy>;
  auto sync = std::make_shared<Sync>(sub_node.get(), SyncPolicy(5), sub1, sub2);

  bool sync_callback_called = false;
  sync->registerCallback(
      [this, &sync_callback_called](const PointCloud2ConstPtr &msg1,
         const PointCloud2ConstPtr &msg2) -> void
      {
        (void) msg1;
        (void) msg2;
        sync_callback_called = true;
        out_pub->publish(*msg1);
      });

  // validation node
  bool val_callback_called = false;
  auto val_sub = val_node->create_subscription<PubInfo>(
      "out/info/pub", qos,
      [this, &val_callback_called](PubInfoPtr pi) -> void
      {
        val_callback_called = true;
        EXPECT_EQ(pi->output_info.header_stamp.sec, 123);
        EXPECT_EQ(pi->output_info.header_stamp.nanosec, 456u);
        EXPECT_EQ(pi->input_infos.size(), 2u);

        std::map<std::string, size_t> topic2idx;
        for(size_t i=0; i<pi->input_infos.size(); i++) {
          topic2idx[pi->input_infos[i].topic_name] = i;
        }

        for(auto ii :pi->input_infos) {
          std::cout << ii.topic_name << std::endl;
          std::cout << ii.sub_time.sec << std::endl;
        }

        EXPECT_EQ(topic2idx.size(), 2u);
        for(size_t i=1; i<=2; i++) {
          auto topic = std::string("/in") + std::to_string(i);
          auto idx = topic2idx[topic];
          const auto& in_info = pi->input_infos[idx];

          switch(i) {
            case 1:
              EXPECT_EQ(in_info.topic_name, "/in1");
              EXPECT_EQ(in_info.sub_time.sec, 123);
              EXPECT_EQ(in_info.sub_time.nanosec, 456u);
              break;
            case 2:
              EXPECT_EQ(in_info.topic_name, "/in2");
              EXPECT_EQ(in_info.sub_time.sec, 124);
              EXPECT_EQ(in_info.sub_time.nanosec, 321u);
              break;
            default:
              // never comes here
              EXPECT_TRUE(false);
          }
        }
      });

  // apply "/clock"
  std::cout << "clock\n";
  send_clock(123, 456);
  spin();
  EXPECT_CLOCK(sub_node->now(), 123, 456u);
  std::cout << "clock\n";

  // PointCloud2 message to publish
  PointCloud2 msg;
  msg.header.stamp = pub_node->now();

  // pub1 & sub
  msg.header.frame_id = 1;
  pub1->publish(msg);
  spin();
  std::cout << "pub1\n";

  // update clock
  send_clock(124, 321);
  spin();
  EXPECT_CLOCK(sub_node->now(), 124, 321);
  std::cout << "clock2\n";

  // pub2
  EXPECT_FALSE(val_callback_called);
  msg.header.frame_id = 2;
  pub2->publish(msg);
  spin();
  std::cout << "pub2\n";

  // verify
  EXPECT_TRUE(sync_callback_called);
  EXPECT_TRUE(val_callback_called);
}

/// TildeSynchronizer only collaborates with TildeSynchronizer
TEST_F(TestSynchronizer, sub_and_tilde_sub)
{
  auto pub1 = pubs[0];
  auto pub2 = pubs[1];
  auto& sub1 = init_and_get_sub<0>();
  Subscriber<PointCloud2> sub2;
  sub2.subscribe(sub_node, "in2", qos.get_rmw_qos_profile());

  // setup sub_node synchronizer
  using SyncPolicy = sync_policies::ExactTime<PointCloud2, PointCloud2>;
  using Sync = TildeSynchronizer<SyncPolicy>;
  auto sync = std::make_shared<Sync>(sub_node.get(), SyncPolicy(5), sub1, sub2);

  bool sync_callback_called = false;
  sync->registerCallback(
      [this, &sync_callback_called](const PointCloud2ConstPtr &msg1,
         const PointCloud2ConstPtr &msg2) -> void
      {
        (void) msg1;
        (void) msg2;
        sync_callback_called = true;
        out_pub->publish(*msg1);
      });

  // validation node
  bool val_callback_called = false;
  auto val_sub = val_node->create_subscription<PubInfo>(
      "out/info/pub", qos,
      [this, &val_callback_called](PubInfoPtr pi) -> void
      {
        val_callback_called = true;
        EXPECT_EQ(pi->output_info.header_stamp.sec, 123);
        EXPECT_EQ(pi->output_info.header_stamp.nanosec, 456u);
        EXPECT_EQ(pi->input_infos.size(), 1u);

        for(const auto &in_info: pi->input_infos) {
          if(in_info.topic_name == "/in1") {
            EXPECT_EQ(in_info.sub_time.sec, 123);
            EXPECT_EQ(in_info.sub_time.nanosec, 456u);
          } else {
            // never comes here
            FAIL() << "invalid topic: " << in_info.topic_name;
          }
        }
      });

  // apply "/clock"
  std::cout << "clock\n";
  send_clock(123, 456);
  spin();
  EXPECT_CLOCK(sub_node->now(), 123, 456u);
  std::cout << "clock\n";

  // PointCloud2 message to publish
  PointCloud2 msg;
  msg.header.stamp = pub_node->now();

  // pub1 & sub
  msg.header.frame_id = 1;
  pub1->publish(msg);
  spin();
  std::cout << "pub1\n";

  // update clock
  send_clock(124, 321);
  spin();
  EXPECT_CLOCK(sub_node->now(), 124, 321);
  std::cout << "clock2\n";

  // pub2
  EXPECT_FALSE(val_callback_called);
  msg.header.frame_id = 2;
  pub2->publish(msg);
  spin();
  std::cout << "pub2\n";

  // verify
  EXPECT_TRUE(sync_callback_called);
  EXPECT_TRUE(val_callback_called);
}


