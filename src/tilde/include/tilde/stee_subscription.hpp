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

#ifndef TILDE__STEE_SUBSCRIPTION_HPP_
#define TILDE__STEE_SUBSCRIPTION_HPP_

#include "rclcpp/subscription.hpp"
#include "tilde/message_conversion.hpp"

#include <cassert>
#include <memory>
#include <string>

namespace tilde
{

#if ROS_DISTRO_GALACTIC
template <
  typename CallbackMessageT,
  typename ConvertedCallbackMessageT = ConvertedMessageType<CallbackMessageT>,
  typename AllocatorT = std::allocator<void>,
  typename MessageMemoryStrategyT =
    rclcpp::message_memory_strategy::MessageMemoryStrategy<CallbackMessageT, AllocatorT>,
  typename ConvertedMessageMemoryStrategyT =
    rclcpp::message_memory_strategy::MessageMemoryStrategy<ConvertedCallbackMessageT, AllocatorT> >
#else
template <
  typename MessageT, typename ConvertedMessageT = ConvertedMessageType<MessageT>,
  typename AllocatorT = std::allocator<void>,
  typename SubscribedT = typename rclcpp::TypeAdapter<MessageT>::custom_type,
  typename ROSMessageT = typename rclcpp::TypeAdapter<MessageT>::ros_message_type,
  typename MessageMemoryStrategyT =
    rclcpp::message_memory_strategy::MessageMemoryStrategy<ROSMessageT, AllocatorT>,
  typename ConvertedMessageMemoryStrategyT =
    rclcpp::message_memory_strategy::MessageMemoryStrategy<ConvertedMessageT, AllocatorT> >
#endif
class SteeSubscription
{
private:
#if ROS_DISTRO_GALACTIC
  using SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT, MessageMemoryStrategyT>;
  using ConvertedSubscriptionT =
    rclcpp::Subscription<ConvertedCallbackMessageT, AllocatorT, ConvertedMessageMemoryStrategyT>;
#else
  using SubscriptionT =
    rclcpp::Subscription<MessageT, AllocatorT, SubscribedT, ROSMessageT, MessageMemoryStrategyT>;
  using ConvertedSubscriptionT = rclcpp::Subscription<
    ConvertedMessageT, AllocatorT, typename rclcpp::TypeAdapter<ConvertedMessageT>::custom_type,
    typename rclcpp::TypeAdapter<ConvertedMessageT>::ros_message_type,
    ConvertedMessageMemoryStrategyT>;
#endif

public:
  RCLCPP_SMART_PTR_DEFINITIONS(SteeSubscription)

  /// Constructor
  /**
   * Hold only one of sub or converted_sub.
   */
  SteeSubscription() {}

  void set_sub(std::shared_ptr<SubscriptionT> sub)
  {
    assert(converted_sub_ == nullptr);
    sub_ = sub;
    topic_fqn_ = sub->get_topic_name();
  }

  void set_converted_sub(std::shared_ptr<ConvertedSubscriptionT> converted_sub)
  {
    assert(sub_ == nullptr);
    converted_sub_ = converted_sub;
    std::string extended_name = converted_sub_->get_topic_name();
    std::string affix = "/stee";
    assert(affix.length() < extended_name.length());
    topic_fqn_ = extended_name.substr(0, extended_name.length() - affix.length());
  }

  [[nodiscard]] RCLCPP_PUBLIC const char * get_topic_name() const { return topic_fqn_.c_str(); }

private:
  std::shared_ptr<SubscriptionT> sub_;
  std::shared_ptr<ConvertedSubscriptionT> converted_sub_;
  std::string topic_fqn_;
};

}  // namespace tilde

#endif  // TILDE__STEE_SUBSCRIPTION_HPP_
