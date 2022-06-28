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

#ifndef TILDE__STEE_PUBLISHER_HPP_
#define TILDE__STEE_PUBLISHER_HPP_

#include <memory>

#include "rclcpp/publisher.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"

#include "tilde/tilde_publisher.hpp"
#include "tilde_msg/msg/stee_source.hpp"

namespace tilde
{

template<typename MessageT,
         typename ConvertedMessageT,
         typename AllocatorT = std::allocator<void>>
class SteePublisher : public TildePublisherBase
{
private:
  using MessageAllocatorTraits = rclcpp::allocator::AllocRebind<MessageT, AllocatorT>;
  using MessageAllocator = typename MessageAllocatorTraits::allocator_type;
  using MessageDeleter = rclcpp::allocator::Deleter<MessageAllocator, MessageT>;
  using PublisherT = rclcpp::Publisher<MessageT, AllocatorT>;
  using ConvertedPublisherT = rclcpp::Publisher<ConvertedMessageT, AllocatorT>;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(SteePublisher)

  /// Default constructor
  SteePublisher(
    std::shared_ptr<PublisherT> pub,
    std::shared_ptr<ConvertedPublisherT> converted_pub,
    const std::string & node_fqn,
    std::shared_ptr<rclcpp::Clock> clock,
    std::shared_ptr<rclcpp::Clock> steady_clock)
  : TildePublisherBase(clock, steady_clock, node_fqn),
    pub_(pub),
    converted_pub_(converted_pub)
  {
  }

  void
  publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    auto converted_msg = std::make_unique<ConvertedMessageT>();
    // Can we avoid copy?
    converted_msg->body = *msg;
    converted_pub_->publish(std::move(converted_msg));
    pub_->publish(std::move(msg));
  }

private:
  std::shared_ptr<PublisherT> pub_;
  std::shared_ptr<ConvertedPublisherT> converted_pub_;
};

}  // namespace tilde

#endif  // TILDE__STEE_PUBLISHER_HPP_
