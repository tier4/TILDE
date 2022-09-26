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

#include "rclcpp/rclcpp.hpp"
#include "tilde/tilde_node.hpp"
#include "tilde/tilde_publisher.hpp"
#include "tilde_msg/msg/message_tracking_tag.hpp"
#include "tilde_msg/msg/test_top_level_stamp.hpp"

#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

using tilde::InputInfo;
using tilde::TildeNode;
using tilde_msg::msg::MessageTrackingTag;

class TestTildeNode : public ::testing::Test
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

std::string str(const builtin_interfaces::msg::Time & time)
{
  std::string s = "";
  s += std::to_string(time.sec);
  s += ".";
  s += std::to_string(time.nanosec);
  return s;
}

/**
 * consider the following system:
 *   sensor_node -> main_node -> checker_node
 *
 * Check main_node MessageTrackingTag stamps by checker_node.
 * The type of message is PointCloud2.
 */
TEST_F(TestTildeNode, simple_case)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  auto sensor_node = std::make_shared<rclcpp::Node>("sensorNode", options);
  auto main_node = std::make_shared<TildeNode>("pubNode", options);
  auto checker_node = std::make_shared<rclcpp::Node>("checkerNode", options);

  auto sensor_pub = sensor_node->create_publisher<sensor_msgs::msg::PointCloud2>("in_topic", 1);
  auto clock_pub = sensor_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

  // apply "/clock"
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock.sec = 123;
  clock_msg.clock.nanosec = 456;

  clock_pub->publish(clock_msg);
  rclcpp::spin_some(sensor_node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(main_node);

  // prepare pub/sub
  auto main_pub = main_node->create_tilde_publisher<sensor_msgs::msg::PointCloud2>("out_topic", 1);
  auto main_sub = main_node->create_tilde_subscription<sensor_msgs::msg::PointCloud2>(
    "in_topic", 1, [&main_pub](sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void {
      std::cout << "main_sub_callback" << std::endl;
      (void)msg;
      main_pub->publish(std::move(msg));
    });

  bool checker_sub_called = false;
  auto checker_sub = checker_node->create_subscription<tilde_msg::msg::MessageTrackingTag>(
    "out_topic/message_tracking_tag", 1,
    [&checker_sub_called,
     clock_msg](tilde_msg::msg::MessageTrackingTag::UniquePtr message_tracking_tag_msg) -> void {
      checker_sub_called = true;
      std::cout << "checker_sub_callback" << std::endl;
      std::cout << "message_tracking_tag_msg: \n"
                << "pub_time: " << str(message_tracking_tag_msg->output_info.pub_time) << "\n"
                << "pub_time_steady: " << str(message_tracking_tag_msg->output_info.pub_time_steady)
                << "\n"
                << std::endl;
      EXPECT_EQ(message_tracking_tag_msg->output_info.pub_time, clock_msg.clock);
      EXPECT_EQ(message_tracking_tag_msg->output_info.has_header_stamp, true);
    });

  // do scenario
  auto sensor_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  sensor_msg->header.stamp = sensor_node->now();
  sensor_pub->publish(std::move(sensor_msg));

  rclcpp::spin_some(sensor_node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(main_node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(checker_node);
  EXPECT_TRUE(checker_sub_called);
}

/**
 * consider the following system:
 *   sensor_node -> main_node -> checker_node
 *
 * Check main_node MessageTrackingTag stamps by checker_node.
 * The type of message is std_msgs::msg::String.
 */
TEST_F(TestTildeNode, no_header_case)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  auto sensor_node = std::make_shared<rclcpp::Node>("sensorNode", options);
  auto main_node = std::make_shared<TildeNode>("pubNode", options);
  auto checker_node = std::make_shared<rclcpp::Node>("checkerNode", options);

  auto sensor_pub = sensor_node->create_publisher<std_msgs::msg::String>("in_topic", 1);
  auto clock_pub = sensor_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

  // apply "/clock"
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock.sec = 123;
  clock_msg.clock.nanosec = 456;

  clock_pub->publish(clock_msg);
  rclcpp::spin_some(sensor_node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(main_node);

  // prepare pub/sub
  auto main_pub = main_node->create_tilde_publisher<std_msgs::msg::String>("out_topic", 1);
  auto main_sub = main_node->create_tilde_subscription<std_msgs::msg::String>(
    "in_topic", 1, [&main_pub](std_msgs::msg::String::UniquePtr msg) -> void {
      std::cout << "main_sub_callback" << std::endl;
      (void)msg;
      main_pub->publish(std::move(msg));
    });

  bool checker_sub_called = false;
  auto checker_sub = checker_node->create_subscription<tilde_msg::msg::MessageTrackingTag>(
    "out_topic/message_tracking_tag", 1,
    [&checker_sub_called,
     clock_msg](tilde_msg::msg::MessageTrackingTag::UniquePtr message_tracking_tag_msg) -> void {
      checker_sub_called = true;
      std::cout << "checker_sub_callback" << std::endl;
      std::cout << "message_tracking_tag_msg: \n"
                << "pub_time: " << str(message_tracking_tag_msg->output_info.pub_time) << "\n"
                << "pub_time_steady: " << str(message_tracking_tag_msg->output_info.pub_time_steady)
                << "\n"
                << std::endl;
      EXPECT_EQ(message_tracking_tag_msg->output_info.pub_time, clock_msg.clock);
      EXPECT_EQ(message_tracking_tag_msg->output_info.has_header_stamp, false);
    });

  // do scenario
  auto msg = std::make_unique<std_msgs::msg::String>();
  sensor_pub->publish(std::move(msg));

  rclcpp::spin_some(sensor_node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(main_node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(checker_node);
  EXPECT_TRUE(checker_sub_called);
}

TEST_F(TestTildeNode, enable_tilde)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  options.append_parameter_override("enable_tilde", false);

  auto sensor_node = std::make_shared<rclcpp::Node>("sensorNode", options);
  auto main_node = std::make_shared<TildeNode>("pubNode", options);
  auto checker_node = std::make_shared<rclcpp::Node>("checkerNode", options);

  bool enable_tilde = true;
  main_node->get_parameter("enable_tilde", enable_tilde);
  EXPECT_EQ(enable_tilde, false);

  auto sensor_pub = sensor_node->create_publisher<sensor_msgs::msg::PointCloud2>("in_topic", 1);
  auto clock_pub = sensor_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

  // apply "/clock"
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock.sec = 123;
  clock_msg.clock.nanosec = 456;

  clock_pub->publish(clock_msg);
  rclcpp::spin_some(sensor_node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(main_node);

  // prepare pub/sub
  auto main_pub = main_node->create_tilde_publisher<sensor_msgs::msg::PointCloud2>("out_topic", 1);
  auto main_sub = main_node->create_tilde_subscription<sensor_msgs::msg::PointCloud2>(
    "in_topic", 1, [&main_pub](sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void {
      std::cout << "main_sub_callback" << std::endl;
      (void)msg;
      main_pub->publish(std::move(msg));
    });

  bool checker_sub_called = false;
  auto checker_sub = checker_node->create_subscription<tilde_msg::msg::MessageTrackingTag>(
    "out_topic/message_tracking_tag", 1,
    [&checker_sub_called,
     clock_msg](tilde_msg::msg::MessageTrackingTag::UniquePtr message_tracking_tag_msg) -> void {
      (void)message_tracking_tag_msg;
      checker_sub_called = true;
      EXPECT_TRUE(false);  // expect never comes here
    });

  // do scenario
  auto sensor_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  sensor_msg->header.stamp = sensor_node->now();
  sensor_pub->publish(std::move(sensor_msg));

  rclcpp::spin_some(sensor_node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(main_node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(checker_node);
  EXPECT_EQ(checker_sub_called, false);
}

TEST_F(TestTildeNode, register_message_as_input_find_subscription_time)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  auto sensor_node = std::make_shared<rclcpp::Node>("sensorNode", options);
  auto main_node = std::make_shared<TildeNode>("mainNode", options);

  auto sensor_pub = sensor_node->create_publisher<sensor_msgs::msg::PointCloud2>("/in_topic", 1);
  auto clock_pub = sensor_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

  // prepare pub/sub
  auto main_pub = main_node->create_tilde_publisher<sensor_msgs::msg::PointCloud2>("/out_topic", 1);
  auto main_sub = main_node->create_tilde_subscription<sensor_msgs::msg::PointCloud2>(
    "/in_topic", 1, [&main_pub](sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void {
      std::cout << "main_sub_callback" << std::endl;
      (void)msg;
      main_pub->publish(std::move(msg));
    });

  using rclcpp::node_interfaces::get_node_topics_interface;
  auto node_topics_interface = get_node_topics_interface(main_node);
  auto in_topic_resolved_name = node_topics_interface->resolve_topic_name("/in_topic");

  // publish @123.456
  rosgraph_msgs::msg::Clock clock_msg1;
  clock_msg1.clock.sec = 123;
  clock_msg1.clock.nanosec = 456;

  clock_pub->publish(clock_msg1);
  for (int i = 0;
       i < 10 || !(main_node->now() == clock_msg1.clock && sensor_node->now() == clock_msg1.clock);
       i++) {
    rclcpp::spin_some(sensor_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    rclcpp::spin_some(main_node);
  }

  EXPECT_EQ(sensor_node->now(), clock_msg1.clock);
  EXPECT_EQ(main_node->now(), clock_msg1.clock);

  sensor_msgs::msg::PointCloud2 sensor_msg1;
  sensor_msg1.header.stamp = sensor_node->now();
  sensor_pub->publish(sensor_msg1);
  rclcpp::spin_some(sensor_node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(main_node);

  // publish @124.321
  rosgraph_msgs::msg::Clock clock_msg2;
  clock_msg2.clock.sec = 124;
  clock_msg2.clock.nanosec = 321;

  clock_pub->publish(clock_msg2);
  for (int i = 0;
       i < 10 || !(main_node->now() == clock_msg2.clock && sensor_node->now() == clock_msg2.clock);
       i++) {
    rclcpp::spin_some(sensor_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    rclcpp::spin_some(main_node);
  }

  EXPECT_EQ(sensor_node->now(), clock_msg2.clock);
  EXPECT_EQ(main_node->now(), clock_msg2.clock);

  sensor_msgs::msg::PointCloud2 sensor_msg2;
  sensor_msg2.header.stamp = sensor_node->now();
  sensor_pub->publish(sensor_msg2);
  rclcpp::spin_some(sensor_node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(main_node);

  // check
  rclcpp::Time subscription_time, subscription_time_steady;
  auto found = main_node->find_subscription_time(
    &sensor_msg1, in_topic_resolved_name, subscription_time, subscription_time_steady);
  EXPECT_TRUE(found);
  builtin_interfaces::msg::Time subscription_time_msg = subscription_time;
  EXPECT_EQ(subscription_time_msg.sec, clock_msg1.clock.sec);
  EXPECT_EQ(subscription_time_msg.nanosec, clock_msg1.clock.nanosec);
}

TEST_F(TestTildeNode, publish_top_level_stamp)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  auto main_node = std::make_shared<TildeNode>("mainNode", options);
  auto checker_node = std::make_shared<rclcpp::Node>("checkerNode", options);

  // prepare publishers
  auto main_pub =
    main_node->create_tilde_publisher<tilde_msg::msg::TestTopLevelStamp>("out_topic", 1);
  auto clock_pub = main_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

  // apply clock
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock.sec = 123;
  clock_msg.clock.nanosec = 456;

  clock_pub->publish(clock_msg);
  rclcpp::spin_some(main_node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(checker_node);

  // prepare checker subscription
  bool checker_sub_called = false;
  auto checker_sub = checker_node->create_subscription<tilde_msg::msg::MessageTrackingTag>(
    "out_topic/message_tracking_tag", 1,
    [&checker_sub_called,
     &clock_msg](tilde_msg::msg::MessageTrackingTag::UniquePtr message_tracking_tag_msg) -> void {
      (void)message_tracking_tag_msg;
      checker_sub_called = true;
      EXPECT_TRUE(message_tracking_tag_msg->output_info.has_header_stamp);
    });

  // publish
  tilde_msg::msg::TestTopLevelStamp msg;
  msg.stamp.sec = 1234;
  msg.stamp.nanosec = 5678;

  main_pub->publish(msg);

  rclcpp::spin_some(main_node);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(checker_node);

  EXPECT_TRUE(checker_sub_called);
}

TEST_F(TestTildeNode, param_enable_tilde)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  auto main_node = std::make_shared<TildeNode>("mainNode", options);
  auto checker_node = std::make_shared<rclcpp::Node>("checkerNode", options);

  auto main_pub =
    main_node->create_tilde_publisher<tilde_msg::msg::TestTopLevelStamp>("out_topic", 1);

  bool checker_sub_called = false;
  auto checker_sub = checker_node->create_subscription<tilde_msg::msg::MessageTrackingTag>(
    "out_topic/message_tracking_tag", 1,
    [&checker_sub_called](
      tilde_msg::msg::MessageTrackingTag::UniquePtr message_tracking_tag_msg) -> void {
      (void)message_tracking_tag_msg;
      checker_sub_called = true;
      EXPECT_TRUE(message_tracking_tag_msg->output_info.has_header_stamp);
    });

  // under enable_tilde = true
  tilde_msg::msg::TestTopLevelStamp msg;
  msg.stamp.sec = 1234;
  msg.stamp.nanosec = 5678;

  main_pub->publish(msg);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(checker_node);

  EXPECT_TRUE(checker_sub_called);

  // under enable_tilde = false
  rclcpp::Parameter param("enable_tilde", false);
  main_node->set_parameter(param);
  rclcpp::spin_some(main_node);
  checker_sub_called = false;

  msg.stamp.sec = 2234;
  msg.stamp.nanosec = 5679;

  main_pub->publish(msg);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(checker_node);

  EXPECT_FALSE(checker_sub_called);
}

class ChildParamCallback : public TildeNode
{
public:
  explicit ChildParamCallback(const rclcpp::NodeOptions & options)
  : TildeNode("child_param_callback", options)
  {
    this->declare_parameter<std::string>("child_param", "original");
    this->get_parameter("child_param", child_param_);

    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        auto result = rcl_interfaces::msg::SetParametersResult();

        result.successful = true;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "child_param") {
            child_param_ = parameter.as_string();
          }
        }
        return result;
      });
  }

  std::string child_param_;

private:
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

TEST_F(TestTildeNode, child_param_callback)
{
  rclcpp::NodeOptions options;

  auto node = std::make_shared<ChildParamCallback>(options);

  // check default
  bool enable_tilde = false;
  node->get_parameter("enable_tilde", enable_tilde);
  EXPECT_TRUE(enable_tilde);

  std::string child_param;
  node->get_parameter("child_param", child_param);
  EXPECT_EQ(child_param, "original");

  // update
  rclcpp::Parameter enable_tilde_update("enable_tilde", false);
  node->set_parameter(enable_tilde_update);
  rclcpp::Parameter child_param_update("child_param", "updated");
  node->set_parameter(child_param_update);

  // check
  node->get_parameter("enable_tilde", enable_tilde);
  EXPECT_FALSE(enable_tilde);

  node->get_parameter("child_param", child_param);
  EXPECT_EQ(child_param, "updated");
}
