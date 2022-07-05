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

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include "rclcpp/publisher.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"

#include "tilde/stee_sources_table.hpp"
#include "tilde/tilde_publisher.hpp"

#include "tilde_msg/msg/stee_source.hpp"

namespace tilde
{
template<typename MessageT,
  typename ConvertedMessageT,
  typename AllocatorT = std::allocator<void>>
class SteePublisher
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
    std::shared_ptr<SteeSourcesTable> source_table,
    std::shared_ptr<PublisherT> pub,
    std::shared_ptr<ConvertedPublisherT> converted_pub,
    const std::string & node_fqn,
    std::shared_ptr<rclcpp::Clock> clock,
    std::shared_ptr<rclcpp::Clock> steady_clock)
  : source_table_(source_table),
    pub_(pub),
    converted_pub_(converted_pub),
    node_fqn_(node_fqn),
    clock_(clock), steady_clock_(steady_clock)
  {
  }

  void
  publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    auto converted_msg = std::make_unique<ConvertedMessageT>();
    // TODO(y-okumura-isp): Can we avoid copy?
    converted_msg->body = *msg;
    set_sources(converted_msg.get());

    converted_pub_->publish(std::move(converted_msg));
    pub_->publish(std::move(msg));
  }

  void
  publish(const MessageT & msg)
  {
    ConvertedMessageT converted_msg;
    // TODO(y-okumura-isp): Can we avoid copy?
    converted_msg.body = msg;
    set_sources(&converted_msg);

    converted_pub_->publish(converted_msg);
    pub_->publish(msg);
  }

  /**
   * publish() variant
   * We can send a main message but cannot send the corresponding MessageTrackingTag
   */
  void
  publish(const rcl_serialized_message_t & serialized_msg)
  {
    std::cout << "publish serialized message (not supported)" << std::endl;
    // publish_info(get_timestamp(clock_->now(), msg.get()));
    pub_->publish(serialized_msg);
  }

  /**
   * publish() variant
   * We can send a main message but cannot send the corresponding MessageTrackingTag
   */
  void
  publish(const rclcpp::SerializedMessage & serialized_msg)
  {
    std::cout << "publish SerializedMessage (not supported)" << std::endl;
    pub_->publish(serialized_msg);
  }

  /**
   * publish() variant
   * We can send a main message but cannot send the corresponding MessageTrackingTag
   */
  void
  publish(rclcpp::LoanedMessage<MessageT, AllocatorT> && loaned_msg)
  {
    std::cout << "publish LoanedMessage (not supported)" << std::endl;
    pub_->publish(loaned_msg);
  }

  size_t
  get_subscription_count() const
  {
    return pub_->get_subscription_count() +
           converted_pub_->get_subscription_count();
  }

  size_t
  get_intra_process_subscription_count() const
  {
    return pub_->get_intra_process_subscription_count() +
           converted_pub_->get_intra_process_subscription_count();
  }

  RCLCPP_PUBLIC
  const char *
  get_topic_name() const
  {
    return pub_->get_topic_name();
  }

  RCLCPP_PUBLIC
  void add_explicit_input_info(
    const std::string & sub_topic,
    const rclcpp::Time & stamp)
  {
    assert(stamp.get_clock_type() == RCL_ROS_TIME);
    is_explicit = true;
    explicit_info_[sub_topic].insert(stamp);
  }

private:
  std::shared_ptr<const SteeSourcesTable> source_table_;
  std::shared_ptr<PublisherT> pub_;
  std::shared_ptr<ConvertedPublisherT> converted_pub_;
  const std::string node_fqn_;
  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<rclcpp::Clock> steady_clock_;

  bool is_explicit{false};
  // explicit input data
  std::map<std::string, std::set<rclcpp::Time>> explicit_info_;

  void set_sources(ConvertedMessageT * converted_msg)
  {
    if (!is_explicit) {
      auto topic_sources = source_table_->get_latest_sources();
      for (auto & topic_source : topic_sources) {
        for (auto & source : topic_source.second) {
          converted_msg->sources.emplace_back(std::move(source));
        }
      }
    } else {
      for (const auto & topic_stamps : explicit_info_) {
        const auto & topic = topic_stamps.first;
        for (const auto & stamp : topic_stamps.second) {
          auto sources = source_table_->get_sources(
            topic,
            stamp);
          for (auto & source : sources) {
            converted_msg->sources.emplace_back(std::move(source));
          }
        }
      }
    }
  }
};

}  // namespace tilde

#endif  // TILDE__STEE_PUBLISHER_HPP_
