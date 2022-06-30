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

#include <memory>

#include "rclcpp/subscription.hpp"

namespace tilde
{

template<
  typename CallbackMessageT,
  typename ConvertedCallbackMessageT,
  typename AllocatorT = std::allocator<void>,
  typename MessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
    CallbackMessageT,
    AllocatorT>,
  typename ConvertedMessageMemoryStrategyT = rclcpp::message_memory_strategy::MessageMemoryStrategy<
    ConvertedCallbackMessageT,
    AllocatorT>
>
class SteeSubscription
{
private:
  using SubscriptionT = rclcpp::Subscription<CallbackMessageT,
      AllocatorT,
      MessageMemoryStrategyT>;
  using ConvertedSubscriptionT = rclcpp::Subscription<ConvertedCallbackMessageT,
      AllocatorT,
      ConvertedMessageMemoryStrategyT>;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(SteeSubscription)

  /// Constructor
  /**
   * Hold only one of sub or converted_sub.
   */
  SteeSubscription()
  {}

  void set_sub(std::shared_ptr<SubscriptionT> sub)
  {
    assert(converted_sub_ == nullptr);
    sub_ = sub;
  }

  void set_converted_sub(std::shared_ptr<ConvertedSubscriptionT> converted_sub)
  {
    assert(sub_ == nullptr);
    converted_sub_ = converted_sub;
  }

private:
  std::shared_ptr<SubscriptionT> sub_;
  std::shared_ptr<ConvertedSubscriptionT> converted_sub_;
};

}  // namespace tilde

#endif  // TILDE__STEE_SUBSCRIPTION_HPP_
