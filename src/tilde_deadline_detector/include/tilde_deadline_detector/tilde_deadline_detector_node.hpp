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

#ifndef TILDE_DEADLINE_DETECTOR__TILDE_DEADLINE_DETECTOR_NODE_HPP_
#define TILDE_DEADLINE_DETECTOR__TILDE_DEADLINE_DETECTOR_NODE_HPP_

#include <gtest/gtest_prod.h>

#include <map>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tilde_msg/msg/pub_info.hpp"

#include "tilde_deadline_detector/forward_estimator.hpp"

namespace tilde_deadline_detector
{
struct PerformanceCounter
{
  void add(float v);

  float avg{0.0};
  float max{0.0};
  uint64_t cnt{0};
};

class TildeDeadlineDetectorNode : public rclcpp::Node
{
  using PubInfoSubscription = rclcpp::Subscription<tilde_msg::msg::PubInfo>;

public:
  RCLCPP_PUBLIC
  explicit TildeDeadlineDetectorNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// see corresponding rclcpp::Node constructor
  RCLCPP_PUBLIC
  explicit TildeDeadlineDetectorNode(
    const std::string & node_name,
    const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_PUBLIC
  explicit TildeDeadlineDetectorNode(const rclcpp::NodeOptions & options);

  RCLCPP_PUBLIC
  virtual ~TildeDeadlineDetectorNode();

  std::set<std::string> get_pub_info_topics() const;

private:
  ForwardEstimator fe;
  std::set<std::string> sensor_topics_;
  std::set<std::string> target_topics_;
  std::map<std::string, int64_t> topic_vs_deadline_ms_;
  uint64_t expire_ms_;
  uint64_t cleanup_ms_;
  bool print_report_{false};

  // work around for no `/clock` bag files.
  // TODO(y-okumura-isp): delete related codes
  rclcpp::Time latest_;

  std::vector<PubInfoSubscription::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;

  PerformanceCounter pubinfo_callback_counter_;
  PerformanceCounter timer_callback_counter_;

  void init();
  void pubinfo_callback(tilde_msg::msg::PubInfo::UniquePtr msg);
};

}  // namespace tilde_deadline_detector

#endif  // TILDE_DEADLINE_DETECTOR__TILDE_DEADLINE_DETECTOR_NODE_HPP_
