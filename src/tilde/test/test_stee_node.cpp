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

#include "tilde/stee_node.hpp"
#include "tilde_msg/msg/stee_point_cloud2.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>

using sensor_msgs::msg::PointCloud2;
using tilde::SteeNode;
using tilde_msg::msg::SteePointCloud2;

class TestSteeNode : public ::testing::Test
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
};

rclcpp::Time get_time(int32_t seconds, uint32_t nanoseconds)
{
  // Use RCL_ROS_TIME because rclcpp::Time(builtin_interfaces::msg::Time) uses
  // RCL_ROS_TIME.
  return rclcpp::Time(seconds, nanoseconds, RCL_ROS_TIME);
}

TEST_F(TestSteeNode, stee_publisher_unique_ptr)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  auto main_node = std::make_shared<SteeNode>("stee_node", options);
  auto stee_pub = main_node->create_stee_publisher<PointCloud2>("topic", 1);

  auto checker_node = std::make_shared<rclcpp::Node>("checker_node");
  bool received_main = false;
  auto main_sub = checker_node->create_subscription<PointCloud2>(
    "topic", 1, [&received_main](PointCloud2::UniquePtr msg) -> void {
      EXPECT_EQ(msg->header.frame_id, "unique");
      received_main = true;
    });
  bool received_converted = false;
  auto converted_sub = checker_node->create_subscription<SteePointCloud2>(
    "topic/stee", 1, [&received_converted](SteePointCloud2::UniquePtr msg) -> void {
      EXPECT_EQ(msg->body.header.frame_id, "unique");
      received_converted = true;
    });

  auto unique_ptr_msg = std::make_unique<PointCloud2>();
  unique_ptr_msg->header.frame_id = "unique";

  stee_pub->publish(std::move(unique_ptr_msg));

  rclcpp::spin_some(checker_node);

  EXPECT_TRUE(received_main);
  EXPECT_TRUE(received_converted);
}

TEST_F(TestSteeNode, stee_publisher_const_reference)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  auto main_node = std::make_shared<SteeNode>("stee_node", options);
  auto stee_pub = main_node->create_stee_publisher<PointCloud2>("topic", 1);

  auto checker_node = std::make_shared<rclcpp::Node>("checker_node");
  bool received_main = false;
  auto main_sub = checker_node->create_subscription<PointCloud2>(
    "topic", 1, [&received_main](const PointCloud2 & msg) -> void {
      EXPECT_EQ(msg.header.frame_id, "unique");
      received_main = true;
    });
  bool received_converted = false;
  auto converted_sub = checker_node->create_subscription<SteePointCloud2>(
    "topic/stee", 1, [&received_converted](const SteePointCloud2 & msg) -> void {
      EXPECT_EQ(msg.body.header.frame_id, "unique");
      received_converted = true;
    });

  PointCloud2 msg;
  msg.header.frame_id = "unique";

  stee_pub->publish(msg);

  rclcpp::spin_some(checker_node);

  EXPECT_TRUE(received_main);
  EXPECT_TRUE(received_converted);
}

TEST_F(TestSteeNode, stee_subscription_unique_ptr)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  auto sensor_node = std::make_shared<SteeNode>("sensor_node", options);
  auto checker_node = std::make_shared<SteeNode>("checker_node", options);
  auto checker_node2 = std::make_shared<rclcpp::Node>("checker_node2", options);
  const std::string test_string = "recv_unique_ptr";

  auto pub = sensor_node->create_stee_publisher<PointCloud2>("topic", 1);

  bool received = false;
  // simulate use defined callback
  auto sub = checker_node->create_stee_subscription<PointCloud2>(
    "topic", 1, [test_string, &received](PointCloud2::UniquePtr msg) -> void {
      received = true;
      EXPECT_EQ(msg->header.frame_id, test_string);
    });

  bool received2 = false;
  auto sub2 = checker_node2->create_subscription<SteePointCloud2>(
    "topic/stee", 1, [test_string, &received2](SteePointCloud2::UniquePtr msg) -> void {
      received2 = true;
      EXPECT_EQ(msg->body.header.frame_id, test_string);
    });

  PointCloud2 msg;
  msg.header.frame_id = test_string;

  pub->publish(msg);
  rclcpp::spin_some(checker_node);
  rclcpp::spin_some(checker_node2);

  EXPECT_TRUE(received);
  EXPECT_TRUE(received2);
}

TEST_F(TestSteeNode, stee_subscription_shared_const)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  auto sensor_node = std::make_shared<SteeNode>("sensor_node", options);
  auto checker_node = std::make_shared<SteeNode>("checker_node", options);
  auto checker_node2 = std::make_shared<rclcpp::Node>("checker_node2", options);
  const std::string test_string = "recv_shared_const";

  auto pub = sensor_node->create_stee_publisher<PointCloud2>("topic", 1);

  bool received = false;
  // simulate use defined callback
  auto sub = checker_node->create_stee_subscription<PointCloud2>(
    "topic", 1, [test_string, &received](std::shared_ptr<const PointCloud2> msg) -> void {
      received = true;
      EXPECT_EQ(msg->header.frame_id, test_string);
    });

  bool received2 = false;
  auto sub2 = checker_node2->create_subscription<SteePointCloud2>(
    "topic/stee", 1, [test_string, &received2](SteePointCloud2::UniquePtr msg) -> void {
      received2 = true;
      EXPECT_EQ(msg->body.header.frame_id, test_string);
    });

  PointCloud2 msg;
  msg.header.frame_id = test_string;

  pub->publish(msg);
  rclcpp::spin_some(checker_node);
  rclcpp::spin_some(checker_node2);

  EXPECT_TRUE(received);
  EXPECT_TRUE(received2);
}

TEST_F(TestSteeNode, one_sub_msg_one_pub)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  auto sensor_node = std::make_shared<SteeNode>("sensor_node", options);
  auto main_node = std::make_shared<SteeNode>("main_node", options);
  auto checker_node = std::make_shared<rclcpp::Node>("checker_node", options);

  auto sensor_pub = sensor_node->create_stee_publisher<PointCloud2>("sensor", 1);

  auto main_pub = main_node->create_stee_publisher<PointCloud2>("main", 1);
  bool main_received = false;
  // simulate use defined callback
  auto main_sub = main_node->create_stee_subscription<PointCloud2>(
    "sensor", 1, [&main_received](std::shared_ptr<const PointCloud2> msg) -> void {
      main_received = true;
      EXPECT_EQ(msg->header.frame_id, "by_sensor");
    });

  bool checker_received = false;
  auto checker_sub = checker_node->create_subscription<SteePointCloud2>(
    "main/stee", 1, [&checker_received](std::shared_ptr<const SteePointCloud2> msg) -> void {
      checker_received = true;
      EXPECT_EQ(msg->body.header.frame_id, "by_main");
      EXPECT_EQ(msg->sources.size(), 1u);
      const auto & source = msg->sources[0];
      EXPECT_EQ(source.topic, "/sensor");
      EXPECT_EQ(source.stamp.sec, 1);
      EXPECT_EQ(source.stamp.nanosec, 100lu);
    });

  // publish sensor
  PointCloud2 msg;
  msg.header.stamp = get_time(1, 100);
  msg.header.frame_id = "by_sensor";

  sensor_pub->publish(msg);
  rclcpp::spin_some(main_node);

  EXPECT_TRUE(main_received);
  EXPECT_FALSE(checker_received);

  // publish main
  msg.header.stamp = get_time(2, 200);
  msg.header.frame_id = "by_main";
  main_pub->publish(msg);

  rclcpp::spin_some(checker_node);
  EXPECT_TRUE(checker_received);
}

TEST_F(TestSteeNode, two_sub_msg_select_latest)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  auto sensor_node = std::make_shared<SteeNode>("sensor_node", options);
  auto main_node = std::make_shared<SteeNode>("main_node", options);
  auto checker_node = std::make_shared<rclcpp::Node>("checker_node", options);

  auto sensor_pub = sensor_node->create_stee_publisher<PointCloud2>("sensor", 1);

  auto main_pub = main_node->create_stee_publisher<PointCloud2>("main", 1);
  bool main_received = false;
  // simulate use defined callback
  auto main_sub = main_node->create_stee_subscription<PointCloud2>(
    "sensor", 1, [&main_received](std::shared_ptr<const PointCloud2> msg) -> void {
      main_received = true;
      EXPECT_EQ(msg->header.frame_id, "by_sensor");
    });

  bool checker_received = false;
  auto checker_sub = checker_node->create_subscription<SteePointCloud2>(
    "main/stee", 1, [&checker_received](std::shared_ptr<const SteePointCloud2> msg) -> void {
      checker_received = true;
      EXPECT_EQ(msg->body.header.frame_id, "by_main");
      EXPECT_EQ(msg->sources.size(), 1u);
      const auto & source = msg->sources[0];
      EXPECT_EQ(source.topic, "/sensor");
      EXPECT_EQ(source.stamp.sec, 1);
      EXPECT_EQ(source.stamp.nanosec, 200lu);
    });

  // publish sensor (old value)
  PointCloud2 msg;
  msg.header.stamp = get_time(1, 100);
  msg.header.frame_id = "by_sensor";

  sensor_pub->publish(msg);
  rclcpp::spin_some(main_node);

  EXPECT_TRUE(main_received);
  EXPECT_FALSE(checker_received);

  // publish sensor (new value)
  main_received = false;

  msg.header.stamp = get_time(1, 200);
  msg.header.frame_id = "by_sensor";

  sensor_pub->publish(msg);
  rclcpp::spin_some(main_node);

  EXPECT_TRUE(main_received);
  EXPECT_FALSE(checker_received);

  // publish main
  msg.header.stamp = get_time(2, 200);
  msg.header.frame_id = "by_main";
  main_pub->publish(msg);

  rclcpp::spin_some(checker_node);
  EXPECT_TRUE(checker_received);
}

TEST_F(TestSteeNode, two_sub_msg_explicit)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  auto sensor_node = std::make_shared<SteeNode>("sensor_node", options);
  auto main_node = std::make_shared<SteeNode>("main_node", options);
  auto checker_node = std::make_shared<rclcpp::Node>("checker_node", options);

  auto sensor_pub = sensor_node->create_stee_publisher<PointCloud2>("sensor", 1);

  auto main_pub = main_node->create_stee_publisher<PointCloud2>("main", 1);
  bool main_received = false;
  // simulate use defined callback
  auto main_sub = main_node->create_stee_subscription<PointCloud2>(
    "sensor", 1, [&main_received](std::shared_ptr<const PointCloud2> msg) -> void {
      main_received = true;
      EXPECT_EQ(msg->header.frame_id, "by_sensor");
    });

  bool checker_received = false;
  auto checker_sub = checker_node->create_subscription<SteePointCloud2>(
    "main/stee", 1, [&checker_received](std::shared_ptr<const SteePointCloud2> msg) -> void {
      checker_received = true;
      EXPECT_EQ(msg->body.header.frame_id, "by_main");
      EXPECT_EQ(msg->sources.size(), 1u);
      const auto & source = msg->sources[0];
      EXPECT_EQ(source.topic, "/sensor");
      EXPECT_EQ(source.stamp.sec, 1);
      EXPECT_EQ(source.stamp.nanosec, 100lu);
    });

  // publish sensor (old value)
  PointCloud2 msg;
  msg.header.stamp = get_time(1, 100);
  msg.header.frame_id = "by_sensor";

  sensor_pub->publish(msg);
  rclcpp::spin_some(main_node);

  EXPECT_TRUE(main_received);
  EXPECT_FALSE(checker_received);

  // publish sensor (new value)
  main_received = false;

  msg.header.stamp = get_time(1, 200);
  msg.header.frame_id = "by_sensor";

  sensor_pub->publish(msg);
  rclcpp::spin_some(main_node);

  EXPECT_TRUE(main_received);
  EXPECT_FALSE(checker_received);

  // publish main with explicit API
  main_pub->add_explicit_input_info("/sensor", get_time(1, 100));

  msg.header.stamp = get_time(2, 200);
  msg.header.frame_id = "by_main";
  main_pub->publish(msg);

  rclcpp::spin_some(checker_node);
  EXPECT_TRUE(checker_received);
}

// https://github.com/tier4/TILDE/issues/61
TEST_F(TestSteeNode, test_non_fqn_topic_explicit_after_publish)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  auto sensor_node = std::make_shared<SteeNode>("sensor_node", options);
  auto main_node = std::make_shared<SteeNode>("main_node", options);
  auto checker_node = std::make_shared<rclcpp::Node>("checker_node", options);

  auto sensor_pub = sensor_node->create_stee_publisher<PointCloud2>("sensor", 1);

  auto main_pub = main_node->create_stee_publisher<PointCloud2>("main", 1);
  bool main_received = false;
  // simulate use defined callback
  auto main_sub = main_node->create_stee_subscription<PointCloud2>(
    "sensor", 1, [&main_received](std::shared_ptr<const PointCloud2> msg) -> void {
      main_received = true;
      EXPECT_EQ(msg->header.frame_id, "by_sensor");
    });

  size_t num_sub = 0;
  bool checker_received = false;
  auto checker_sub = checker_node->create_subscription<SteePointCloud2>(
    "main/stee", 1,
    [&num_sub, &checker_received](std::shared_ptr<const SteePointCloud2> msg) -> void {
      if (num_sub == 0) {
        checker_received = true;
        EXPECT_EQ(msg->body.header.frame_id, "by_main");
        EXPECT_EQ(msg->sources.size(), 1u);
        const auto & source = msg->sources[0];
        EXPECT_EQ(source.topic, "/sensor");
        EXPECT_EQ(source.stamp.sec, 1);
        EXPECT_EQ(source.stamp.nanosec, 100lu);
        ++num_sub;
      } else if (num_sub == 1) {
        checker_received = true;
        EXPECT_EQ(msg->body.header.frame_id, "by_main");
        EXPECT_EQ(msg->sources.size(), 1u);
        const auto & source = msg->sources[0];
        EXPECT_EQ(source.topic, "/sensor");
        EXPECT_EQ(source.stamp.sec, 2);
        EXPECT_EQ(source.stamp.nanosec, 200lu);
        ++num_sub;
      }
    });

  // publish sensor1
  PointCloud2 msg;
  msg.header.stamp = get_time(1, 100);
  msg.header.frame_id = "by_sensor";

  sensor_pub->publish(msg);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(main_node);

  main_pub->add_explicit_input_info(main_sub->get_topic_name(), get_time(1, 100));
  msg.header.stamp = get_time(1, 100);
  msg.header.frame_id = "by_main";
  main_pub->publish(msg);

  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(checker_node);

  EXPECT_TRUE(main_received);
  EXPECT_TRUE(checker_received);
  EXPECT_EQ(num_sub, 1u);

  // publish sensor2
  main_received = false;
  checker_received = false;

  msg.header.stamp = get_time(2, 200);
  msg.header.frame_id = "by_sensor";

  sensor_pub->publish(msg);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(main_node);

  main_pub->add_explicit_input_info(main_sub->get_topic_name(), get_time(2, 200));
  msg.header.stamp = get_time(2, 200);
  msg.header.frame_id = "by_main";
  main_pub->publish(msg);

  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(checker_node);

  EXPECT_TRUE(main_received);
  EXPECT_TRUE(checker_received);
  EXPECT_EQ(num_sub, 2u);
}

TEST_F(TestSteeNode, two_topics_one_pub)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  auto sensor_node = std::make_shared<SteeNode>("sensor_node", options);
  auto main_node = std::make_shared<SteeNode>("main_node", options);
  auto checker_node = std::make_shared<rclcpp::Node>("checker_node2", options);
  const std::string test_string = "recv_shared_const";

  auto sensor1_pub = sensor_node->create_stee_publisher<PointCloud2>("sensor1", 1);
  auto sensor2_pub = sensor_node->create_stee_publisher<PointCloud2>("sensor2", 1);

  auto main_pub = main_node->create_stee_publisher<PointCloud2>("main", 1);

  // sensor subscription
  bool sensor1_received = false;
  auto sensor1_sub = main_node->create_stee_subscription<PointCloud2>(
    "sensor1", 1, [test_string, &sensor1_received](std::shared_ptr<const PointCloud2> msg) -> void {
      sensor1_received = true;
      EXPECT_EQ(msg->header.frame_id, "by_sensor1");
    });
  bool sensor2_received = false;
  auto sensor2_sub = main_node->create_stee_subscription<PointCloud2>(
    "sensor2", 1, [test_string, &sensor2_received](std::shared_ptr<const PointCloud2> msg) -> void {
      sensor2_received = true;
      EXPECT_EQ(msg->header.frame_id, "by_sensor2");
    });

  // checker subscription
  bool checker_received = false;
  auto checker_sub = checker_node->create_subscription<SteePointCloud2>(
    "main/stee", 1,
    [test_string, &checker_received](std::shared_ptr<const SteePointCloud2> msg) -> void {
      checker_received = true;
      EXPECT_EQ(msg->body.header.frame_id, "by_main");
      EXPECT_EQ(msg->sources.size(), 2u);
      {
        const auto & source = msg->sources[0];
        EXPECT_EQ(source.topic, "/sensor1");
        EXPECT_EQ(source.stamp.sec, 1);
        EXPECT_EQ(source.stamp.nanosec, 100lu);
      }
      {
        const auto & source = msg->sources[1];
        EXPECT_EQ(source.topic, "/sensor2");
        EXPECT_EQ(source.stamp.sec, 1);
        EXPECT_EQ(source.stamp.nanosec, 200lu);
      }
    });

  // publish sensor1
  PointCloud2 msg;
  msg.header.stamp = get_time(1, 100);
  msg.header.frame_id = "by_sensor1";

  sensor1_pub->publish(msg);
  rclcpp::spin_some(main_node);

  EXPECT_TRUE(sensor1_received);
  EXPECT_FALSE(sensor2_received);
  EXPECT_FALSE(checker_received);

  // publish sensor2p
  msg.header.stamp = get_time(1, 200);
  msg.header.frame_id = "by_sensor2";

  sensor2_pub->publish(msg);
  rclcpp::spin_some(main_node);

  EXPECT_TRUE(sensor1_received);
  EXPECT_TRUE(sensor2_received);
  EXPECT_FALSE(checker_received);

  // publish main
  msg.header.stamp = get_time(2, 200);
  msg.header.frame_id = "by_main";
  main_pub->publish(msg);

  rclcpp::spin_some(checker_node);
  EXPECT_TRUE(checker_received);
}

TEST_F(TestSteeNode, param_enable_stee)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  options.append_parameter_override("enable_stee", false);

  // if not enable_stee, we can communicate non-stee publisher and
  // stee subscription
  auto sensor_node = std::make_shared<SteeNode>("sensor_node", options);
  auto main_node = std::make_shared<SteeNode>("main_node", options);

  auto non_stee_pub = sensor_node->create_publisher<PointCloud2>("topic", 1);

  bool got_message = false;
  auto main_stee_sub = main_node->create_stee_subscription<PointCloud2>(
    "topic", 1, [&got_message](std::shared_ptr<const PointCloud2> msg) -> void {
      (void)msg;
      got_message = true;
    });

  // publish sensor
  PointCloud2 msg;
  msg.header.stamp = get_time(1, 100);
  msg.header.frame_id = "by_sensor1";
  non_stee_pub->publish(msg);

  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(main_node);
  EXPECT_TRUE(got_message);
}

TEST_F(TestSteeNode, remove_duplicated_sources)
{
  /* DAG is
   *   A --> B --> C -->
   *   |           ^
   *   +-----------+
   *
   * Check C publishes unique A entry although the same message of A is used by B, and C.
   * Originally, A was listed twice.
   */

  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);

  auto node_a = std::make_shared<SteeNode>("nodeA", options);
  auto node_b = std::make_shared<SteeNode>("nodeB", options);
  auto node_c = std::make_shared<SteeNode>("nodeC", options);
  auto checker_node = std::make_shared<rclcpp::Node>("checker_node", options);

  // nodeA directly publish SteePointCloud2 to fill sources field manually
  auto pub_a = node_a->create_publisher<SteePointCloud2>("a/stee", 1);

  // subscriptions of A
  bool get_b_a{false};
  auto sub_b_a = node_b->create_stee_subscription<PointCloud2>(
    "a", 1, [&get_b_a](std::shared_ptr<const PointCloud2> msg) {
      (void)msg;
      get_b_a = true;
    });
  bool get_c_a{false};
  auto sub_c_a = node_c->create_stee_subscription<PointCloud2>(
    "a", 1, [&get_c_a](std::shared_ptr<const PointCloud2> msg) {
      (void)msg;
      get_c_a = true;
    });

  auto pub_b = node_b->create_stee_publisher<PointCloud2>("b", 1);

  auto pub_c = node_c->create_stee_publisher<PointCloud2>("c", 1);
  bool get_c_b{false};
  auto sub_c_b = node_c->create_stee_subscription<PointCloud2>(
    "b", 1, [&pub_c, &get_c_b](std::shared_ptr<const PointCloud2> msg) {
      get_c_b = true;
      pub_c->publish(*msg);
    });

  bool checker_node_called{false};
  auto sub_checker = checker_node->create_subscription<SteePointCloud2>(
    "c/stee", 1, [&checker_node_called](std::shared_ptr<const SteePointCloud2> msg) {
      checker_node_called = true;
      EXPECT_EQ(msg->sources.size(), 1u);
      EXPECT_EQ(msg->sources[0].topic, "in");
      EXPECT_EQ(msg->sources[0].stamp.sec, 8);
      EXPECT_EQ(msg->sources[0].stamp.nanosec, 18u);
      EXPECT_EQ(msg->sources[0].first_subscription_steady_time.sec, 5);
      EXPECT_EQ(msg->sources[0].first_subscription_steady_time.nanosec, 15u);
    });

  SteePointCloud2 msg_a;
  msg_a.body.header.stamp = get_time(10, 110);
  tilde_msg::msg::SteeSource source;
  source.topic = "in";
  source.stamp = get_time(8, 18);
  source.first_subscription_steady_time = get_time(5, 15);
  msg_a.sources.push_back(source);

  pub_a->publish(msg_a);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(node_b);
  rclcpp::spin_some(node_c);

  PointCloud2 msg_b;
  msg_b.header.stamp = msg_a.body.header.stamp;
  pub_b->publish(msg_b);
  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(node_c);

  rclcpp::Rate(50).sleep();
  rclcpp::spin_some(checker_node);

  EXPECT_TRUE(get_b_a);
  EXPECT_TRUE(get_c_a);
  EXPECT_TRUE(get_c_b);
  EXPECT_TRUE(checker_node_called);
}
