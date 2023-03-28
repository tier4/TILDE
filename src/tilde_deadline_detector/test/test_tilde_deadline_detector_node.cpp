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

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tilde/tilde_node.hpp"
#include "tilde_deadline_detector/tilde_deadline_detector_node.hpp"
#include "tilde_msg/msg/message_tracking_tag.hpp"
#include "tilde_msg/msg/sub_topic_time_info.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using tilde_deadline_detector::TildeDeadlineDetectorNode;

class TestTildeDeadlineDetectorNode : public ::testing::Test
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

builtin_interfaces::msg::Time get_time(int sec, int nsec)
{
  builtin_interfaces::msg::Time t;
  t.sec = sec;
  t.nanosec = nsec;
  return t;
}

builtin_interfaces::msg::Duration get_duration(int32_t sec, uint32_t nsec)
{
  return rclcpp::Duration(sec, nsec);
}

TEST_F(TestTildeDeadlineDetectorNode, get_message_tracking_tag_topics)
{
  auto tilde_node = tilde::TildeNode("tilde_node");
  auto pub = tilde_node.create_tilde_publisher<sensor_msgs::msg::PointCloud2>("topic", 1);
  auto pub2 = tilde_node.create_publisher<sensor_msgs::msg::PointCloud2>("topic2", 1);
  auto pub3 =
    tilde_node.create_publisher<sensor_msgs::msg::PointCloud2>("topic2/message_tracking_tag", 1);

  auto det_node = TildeDeadlineDetectorNode("node");
  auto topics = det_node.get_message_tracking_tag_topics();

  EXPECT_EQ(topics.size(), 1u);
  EXPECT_EQ(*topics.begin(), "/topic/message_tracking_tag");
}

tilde_msg::msg::MessageTrackingTag get_message_tracking_tag(
  const std::string & topic, builtin_interfaces::msg::Time stamp,
  builtin_interfaces::msg::Time pub_steady)
{
  tilde_msg::msg::MessageTrackingTag tag;
  tag.output_info.topic_name = topic;
  tag.output_info.pub_time_steady = pub_steady;
  tag.output_info.has_header_stamp = true;
  tag.output_info.header_stamp = stamp;
  return tag;
}

void add_input_info(
  tilde_msg::msg::MessageTrackingTag & tag, const tilde_msg::msg::MessageTrackingTag & in_tag)
{
  tilde_msg::msg::SubTopicTimeInfo sub_info;
  sub_info.topic_name = in_tag.output_info.topic_name;
  sub_info.has_header_stamp = true;
  sub_info.header_stamp = in_tag.output_info.header_stamp;
  tag.input_infos.push_back(sub_info);
}

TEST_F(TestTildeDeadlineDetectorNode, test_deadline_detection)
{
  auto tag_sender_node = std::make_shared<rclcpp::Node>("tag_sender_node");
  auto sensor_pub = tag_sender_node->create_publisher<tilde_msg::msg::MessageTrackingTag>(
    "sensor/message_tracking_tag", rclcpp::QoS(1).best_effort());
  auto next_pub = tag_sender_node->create_publisher<tilde_msg::msg::MessageTrackingTag>(
    "next/message_tracking_tag", rclcpp::QoS(1).best_effort());

  rclcpp::NodeOptions options;
  options.append_parameter_override("target_topics", std::vector<std::string>{"next"});
  int deadline_ms = 10;
  options.append_parameter_override("deadline_ms", std::vector<int>{deadline_ms});

  auto deadline_detector_node =
    std::make_shared<TildeDeadlineDetectorNode>("deadline_detector_node", options);

  auto checker_node = std::make_shared<rclcpp::Node>("checker_node");
  bool notification_called = false;
  auto checker_sub = checker_node->create_subscription<tilde_msg::msg::DeadlineNotification>(
    "deadline_notification", rclcpp::QoS(1).best_effort(),
    [this, &notification_called,
     deadline_ms](tilde_msg::msg::DeadlineNotification::UniquePtr msg) -> void {
      notification_called = true;
      EXPECT_EQ(msg->topic_name, "next");
      EXPECT_EQ(msg->stamp, get_time(200, 0));
      EXPECT_EQ(msg->deadline_setting, get_duration(0, deadline_ms * 1000 * 1000));
      auto & sources = msg->sources;
      EXPECT_EQ(sources.size(), 1u);
      auto & s = sources[0];
      EXPECT_EQ(s.topic, "sensor");
      EXPECT_EQ(s.stamp, get_time(200, 0));
      EXPECT_EQ(s.elapsed, get_duration(0, deadline_ms * 1000 * 1000));
      EXPECT_TRUE(s.is_overrun);
    });

  auto spin = [tag_sender_node, deadline_detector_node, checker_node]() -> void {
    rclcpp::spin_some(tag_sender_node);
    rclcpp::spin_some(deadline_detector_node);
    rclcpp::spin_some(checker_node);
  };

  // case1: not overrun
  {
    auto t1 = get_time(100, 0);
    auto t2 = get_time(100, deadline_ms * 1000 * 1000 - 1);  // ms to ns
    tilde_msg::msg::MessageTrackingTag sensor_tag = get_message_tracking_tag("sensor", t1, t1);

    tilde_msg::msg::MessageTrackingTag next_tag = get_message_tracking_tag("next", t1, t2);
    add_input_info(next_tag, sensor_tag);

    sensor_pub->publish(sensor_tag);
    spin();

    next_pub->publish(next_tag);
    spin();

    EXPECT_FALSE(notification_called);
  }

  // case2: overrun
  {
    auto t1 = get_time(200, 0);
    auto t2 = get_time(200, deadline_ms * 1000 * 1000);  // ms to ns
    tilde_msg::msg::MessageTrackingTag sensor_tag = get_message_tracking_tag("sensor", t1, t1);

    tilde_msg::msg::MessageTrackingTag next_tag = get_message_tracking_tag("next", t1, t2);
    add_input_info(next_tag, sensor_tag);

    sensor_pub->publish(sensor_tag);
    spin();

    next_pub->publish(next_tag);
    spin();

    EXPECT_TRUE(notification_called);

    // we check deadline notification message in subscription callback
  }
}

TEST_F(TestTildeDeadlineDetectorNode, test_deadline_detection_multiple_input)
{
  auto tag_sender_node = std::make_shared<rclcpp::Node>("tag_sender_node");
  auto sensor1_pub = tag_sender_node->create_publisher<tilde_msg::msg::MessageTrackingTag>(
    "sensor1/message_tracking_tag", rclcpp::QoS(1).best_effort());
  auto sensor2_pub = tag_sender_node->create_publisher<tilde_msg::msg::MessageTrackingTag>(
    "sensor2/message_tracking_tag", rclcpp::QoS(1).best_effort());
  auto next_pub = tag_sender_node->create_publisher<tilde_msg::msg::MessageTrackingTag>(
    "next/message_tracking_tag", rclcpp::QoS(1).best_effort());

  rclcpp::NodeOptions options;
  options.append_parameter_override("target_topics", std::vector<std::string>{"next"});
  int deadline_ms = 10;
  options.append_parameter_override("deadline_ms", std::vector<int>{deadline_ms});

  auto deadline_detector_node =
    std::make_shared<TildeDeadlineDetectorNode>("deadline_detector_node", options);

  auto checker_node = std::make_shared<rclcpp::Node>("checker_node");
  bool notification_called = false;
  auto checker_sub = checker_node->create_subscription<tilde_msg::msg::DeadlineNotification>(
    "deadline_notification", rclcpp::QoS(1).best_effort(),
    [this, &notification_called,
     deadline_ms](tilde_msg::msg::DeadlineNotification::UniquePtr msg) -> void {
      notification_called = true;
      EXPECT_EQ(msg->topic_name, "next");
      EXPECT_EQ(msg->stamp, get_time(200, 0));
      EXPECT_EQ(msg->deadline_setting, get_duration(0, deadline_ms * 1000 * 1000));

      auto & sources = msg->sources;
      EXPECT_EQ(sources.size(), 2u);

      size_t sensor1_idx = 0;
      size_t sensor2_idx = 1;
      if (sources[sensor1_idx].topic == "sensor2") {
        sensor1_idx = 1;
        sensor2_idx = 0;
      }

      auto & s1 = sources[sensor1_idx];
      EXPECT_EQ(s1.topic, "sensor1");
      EXPECT_EQ(s1.stamp, get_time(200, 0));
      EXPECT_EQ(s1.elapsed, get_duration(0, deadline_ms * 1000 * 1000));
      EXPECT_TRUE(s1.is_overrun);

      auto & s2 = sources[sensor2_idx];
      EXPECT_EQ(s2.topic, "sensor2");
      EXPECT_EQ(s2.stamp, get_time(200, (deadline_ms / 2) * 1000 * 1000));
      EXPECT_EQ(s2.elapsed, get_duration(0, (deadline_ms / 2) * 1000 * 1000));
      EXPECT_FALSE(s2.is_overrun);
    });

  auto spin = [tag_sender_node, deadline_detector_node, checker_node]() -> void {
    rclcpp::spin_some(tag_sender_node);
    rclcpp::spin_some(deadline_detector_node);
    rclcpp::spin_some(checker_node);
  };

  // case1: not overrun
  {
    auto t1 = get_time(100, 0);
    auto t2 = get_time(100, deadline_ms * 1000 * 1000 - 1);  // ms to ns
    tilde_msg::msg::MessageTrackingTag sensor_tag = get_message_tracking_tag("sensor1", t1, t1);

    tilde_msg::msg::MessageTrackingTag next_tag = get_message_tracking_tag("next", t1, t2);
    add_input_info(next_tag, sensor_tag);

    sensor1_pub->publish(sensor_tag);
    spin();

    next_pub->publish(next_tag);
    spin();

    EXPECT_FALSE(notification_called);
  }

  // case2: overrun. There are 2 inputs, and sensor1 is overrun.
  {
    auto t1 = get_time(200, 0);
    auto t2 = get_time(200, (deadline_ms / 2) * 1000 * 1000);
    auto t3 = get_time(200, deadline_ms * 1000 * 1000);

    tilde_msg::msg::MessageTrackingTag sensor1_tag = get_message_tracking_tag("sensor1", t1, t1);
    tilde_msg::msg::MessageTrackingTag sensor2_tag = get_message_tracking_tag("sensor2", t2, t2);

    tilde_msg::msg::MessageTrackingTag next_tag = get_message_tracking_tag("next", t1, t3);
    add_input_info(next_tag, sensor1_tag);
    add_input_info(next_tag, sensor2_tag);

    sensor1_pub->publish(sensor1_tag);
    spin();

    sensor2_pub->publish(sensor2_tag);
    spin();

    next_pub->publish(next_tag);
    spin();

    EXPECT_TRUE(notification_called);

    // we check deadline notification message in subscription callback
  }
}
