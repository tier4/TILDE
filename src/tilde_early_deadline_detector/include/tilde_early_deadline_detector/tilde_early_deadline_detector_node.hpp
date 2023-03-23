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

#ifndef TILDE_EARLY_DEADLINE_DETECTOR__TILDE_EARLY_DEADLINE_DETECTOR_NODE_HPP_
#define TILDE_EARLY_DEADLINE_DETECTOR__TILDE_EARLY_DEADLINE_DETECTOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tilde_early_deadline_detector/forward_estimator.hpp"
// #include "tilde_deadline_detector/tilde_deadline_detector_node.hpp"
#include "tilde_msg/msg/deadline_notification.hpp"
#include "tilde_msg/msg/message_tracking_tag.hpp"

#include <gtest/gtest_prod.h>

#include <map>
#include <set>
#include <string>
#include <vector>

// map header
#include <iostream>


namespace tilde_early_deadline_detector
{
struct PerformanceCounter
{
  void add(float v);

  float avg{0.0};
  float max{0.0};
  uint64_t cnt{0};
};

class TildeEarlyDeadlineDetectorNode : public rclcpp::Node
{
  using MessageTrackingTagSubscription = rclcpp::Subscription<tilde_msg::msg::MessageTrackingTag>;

public:
  RCLCPP_PUBLIC
  explicit TildeEarlyDeadlineDetectorNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// see corresponding rclcpp::Node constructor
  RCLCPP_PUBLIC
  explicit TildeEarlyDeadlineDetectorNode(
    const std::string & node_name, const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_PUBLIC
  explicit TildeEarlyDeadlineDetectorNode(const rclcpp::NodeOptions & options);

  RCLCPP_PUBLIC
  virtual ~TildeEarlyDeadlineDetectorNode();

  std::set<std::string> get_message_tracking_tag_topics() const;

private:
  ForwardEstimator fe;
  std::set<std::string> sensor_topics_;
  std::set<std::string> target_topics_;
  std::set<std::string> end_topics_;
  std::map<std::string, int64_t> topic_vs_deadline_ms_;

  uint64_t expire_ms_;
  uint64_t cleanup_ms_;
  bool print_report_{false};
  bool print_pending_messages_{false};

  // work around for no `/clock` bag files.
  // TODO(y-okumura-isp): delete related codes
  rclcpp::Time latest_;

  std::vector<MessageTrackingTagSubscription::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<tilde_msg::msg::DeadlineNotification>::SharedPtr notification_pub_;

 PerformanceCounter message_tracking_tag_callback_counter_;
 PerformanceCounter timer_callback_counter_;

  void init();
  void message_tracking_tag_callback(tilde_msg::msg::MessageTrackingTag::UniquePtr msg);
};


/// change here
/// changed constructor name
// TildeEarlyDeadlineDetectorNode inherits TildeDeadlineDetectorNode
// override the part differs from TildeDeadlineDetectorNode
// delete the same code(compare to TildeDeadlineDetectorNode) later
// class TildeEarlyDeadlineDetectorNode : public tilde_deadline_detector::TildeDeadlineDetectorNode{
//   using MessageTrackingTagSubscription = rclcpp::Subscription<tilde_msg::msg::MessageTrackingTag>;

// public:
//   // constructors
//   RCLCPP_PUBLIC
//   explicit TildeEarlyDeadlineDetectorNode(
//     const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

//   /// see corresponding rclcpp::Node constructor
//   RCLCPP_PUBLIC
//   explicit TildeEarlyDeadlineDetectorNode(
//     const std::string & node_name, const std::string & namespace_,
//     const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

//   RCLCPP_PUBLIC
//   explicit TildeEarlyDeadlineDetectorNode(const rclcpp::NodeOptions & options);

//   RCLCPP_PUBLIC
//   virtual ~TildeEarlyDeadlineDetectorNode();

//   std::set<std::string> get_message_tracking_tag_topics() const;

// private:
//   tilde_deadline_detector::ForwardEstimator fe;
//   std::set<std::string> sensor_topics_;
//   std::set<std::string> target_topics_;
//   std::set<std::string> end_topics_;
//   std::map<std::string, int64_t> topic_vs_deadline_ms_;

//   uint64_t expire_ms_;
//   uint64_t cleanup_ms_;
//   bool print_report_{false};
//   bool print_pending_messages_{false};

//   // work around for no `/clock` bag files.
//   // TODO(y-okumura-isp): delete related codes
//   rclcpp::Time latest_;

//   std::vector<MessageTrackingTagSubscription::SharedPtr> subs_;
//   rclcpp::TimerBase::SharedPtr timer_;

//   rclcpp::Publisher<tilde_msg::msg::DeadlineNotification>::SharedPtr notification_pub_;

//   tilde_deadline_detector::PerformanceCounter message_tracking_tag_callback_counter_;
//   tilde_deadline_detector::PerformanceCounter timer_callback_counter_;

//   // main function
//   void init();
//   void message_tracking_tag_callback(tilde_msg::msg::MessageTrackingTag::UniquePtr msg);
// };

}  // namespace tilde_deadline_detector

#endif  // TILDE_EARLY_DEADLINE_DETECTOR__TILDE_DEADLINE_DETECTOR_NODE_HPP_
