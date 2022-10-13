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

#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/publisher.hpp"
#include "tilde/message_conversion.hpp"
#include "tilde/stee_sources_table.hpp"
#include "tilde/tilde_publisher.hpp"
#include "tilde_msg/msg/stee_source.hpp"

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>

namespace tilde
{
template <
  typename MessageT, typename ConvertedMessageT = ConvertedMessageType<MessageT>,
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
    std::shared_ptr<SteeSourcesTable> source_table, std::shared_ptr<PublisherT> pub,
    std::shared_ptr<ConvertedPublisherT> converted_pub, const std::string & node_fqn,
    std::shared_ptr<rclcpp::Clock> clock, std::shared_ptr<rclcpp::Clock> steady_clock)
  : source_table_(source_table),
    pub_(pub),
    converted_pub_(converted_pub),
    node_fqn_(node_fqn),
    clock_(clock),
    steady_clock_(steady_clock)
  {
  }

  void publish(std::unique_ptr<MessageT, MessageDeleter> msg)
  {
    auto converted_msg = std::make_unique<ConvertedMessageT>();
    // TODO(y-okumura-isp): Can we avoid copy?
    converted_msg->body = *msg;
    set_sources(converted_msg.get());

    converted_pub_->publish(std::move(converted_msg));

    if (pub_) {
      pub_->publish(std::move(msg));
    }
  }

  void publish(const MessageT & msg)
  {
    ConvertedMessageT converted_msg;
    // TODO(y-okumura-isp): Can we avoid copy?
    converted_msg.body = msg;
    set_sources(&converted_msg);

    converted_pub_->publish(converted_msg);

    if (pub_) {
      pub_->publish(msg);
    }
  }

  /**
   * publish() variant
   * We can send a main message but cannot send the corresponding MessageTrackingTag
   */
  void publish(const rcl_serialized_message_t & serialized_msg)
  {
    std::cout << "publish serialized message (not supported)" << std::endl;
    // publish_info(get_timestamp(clock_->now(), msg.get()));
    if (pub_) {
      pub_->publish(serialized_msg);
    }
  }

  /**
   * publish() variant
   * We can send a main message but cannot send the corresponding MessageTrackingTag
   */
  void publish(const rclcpp::SerializedMessage & serialized_msg)
  {
    std::cout << "publish SerializedMessage (not supported)" << std::endl;
    if (pub_) {
      pub_->publish(serialized_msg);
    }
  }

  /**
   * publish() variant
   * We can send a main message but cannot send the corresponding MessageTrackingTag
   */
  void publish(rclcpp::LoanedMessage<MessageT, AllocatorT> && loaned_msg)
  {
    std::cout << "publish LoanedMessage (not supported)" << std::endl;
    if (pub_) {
      pub_->publish(loaned_msg);
    }
  }

  size_t get_subscription_count() const
  {
    size_t ret = 0;
    if (pub_) {
      ret += pub_->get_subscription_count();
    }

    return ret + converted_pub_->get_subscription_count();
  }

  size_t get_intra_process_subscription_count() const
  {
    size_t ret = 0;
    if (pub_) {
      ret = pub_->get_intra_process_subscription_count();
    }
    return ret + converted_pub_->get_intra_process_subscription_count();
  }

  RCLCPP_PUBLIC
  const char * get_topic_name() const
  {
    if (pub_) {
      return pub_->get_topic_name();
    }

    return converted_pub_->get_topic_name();
  }

  /// Explicit API
  /**
   * \param[in] sub_topic topic FQN
   * \param[in] stamp header.stamp of the used message
   */
  RCLCPP_PUBLIC
  void add_explicit_input_info(const std::string & sub_topic, const rclcpp::Time & stamp)
  {
    assert(stamp.get_clock_type() == RCL_ROS_TIME);
    is_explicit_ = true;
    explicit_info_[sub_topic].insert(stamp);
  }

private:
  std::shared_ptr<const SteeSourcesTable> source_table_;
  std::shared_ptr<PublisherT> pub_;
  std::shared_ptr<ConvertedPublisherT> converted_pub_;
  const std::string node_fqn_;
  std::shared_ptr<rclcpp::Clock> clock_;
  std::shared_ptr<rclcpp::Clock> steady_clock_;

  bool is_explicit_{false};
  // explicit input data
  std::map<std::string, std::set<rclcpp::Time>> explicit_info_;

  void set_sources(ConvertedMessageT * converted_msg)
  {
    std::set<tilde_msg::msg::SteeSource, SteeSourceCmp> found;

    if (!is_explicit_) {
      auto topic_sources = source_table_->get_latest_sources();
      for (auto & topic_source : topic_sources) {
        for (auto & source : topic_source.second) {
          if (found.find(source) != found.end()) {
            continue;
          }
          found.insert(source);
          converted_msg->sources.emplace_back(std::move(source));
        }
      }
    } else {
      for (const auto & topic_stamps : explicit_info_) {
        const auto & topic = topic_stamps.first;
        for (const auto & stamp : topic_stamps.second) {
          auto sources = source_table_->get_sources(topic, stamp);
          for (auto & source : sources) {
            if (found.find(source) != found.end()) {
              continue;
            }
            found.insert(source);
            converted_msg->sources.emplace_back(std::move(source));
          }
        }
      }
      explicit_info_.clear();
    }
  }
};

}  // namespace tilde

#endif  // TILDE__STEE_PUBLISHER_HPP_
