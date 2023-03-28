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

#ifndef TILDE__STEE_NODE_HPP_
#define TILDE__STEE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tilde/message_conversion.hpp"
#include "tilde/stee_publisher.hpp"
#include "tilde/stee_sources_table.hpp"
#include "tilde/stee_subscription.hpp"
#include "tilde_msg/msg/stee_source.hpp"

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace tilde
{
template <class>
inline constexpr bool always_false_v = false;

class SteeNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SteeNode)

  /// see corresponding rclcpp::Node constructor
  RCLCPP_PUBLIC
  explicit SteeNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// see corresponding rclcpp::Node constructor
  RCLCPP_PUBLIC
  explicit SteeNode(
    const std::string & node_name, const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_PUBLIC
  virtual ~SteeNode();

  template <
    typename MessageT, typename ConvertedMessageT = ConvertedMessageType<MessageT>,
    typename CallbackT, typename AllocatorT = std::allocator<void>,
    typename MessageDeleter = std::default_delete<MessageT>,
    typename CallbackMessageT =
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
    typename CallbackArgT =
      typename rclcpp::function_traits::function_traits<CallbackT>::template argument_type<0>,
    typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>,
    typename ConvertedSubscriptionT = rclcpp::Subscription<ConvertedMessageT, AllocatorT>,
    typename MessageMemoryStrategyT =
      rclcpp::message_memory_strategy::MessageMemoryStrategy<CallbackMessageT, AllocatorT>,
    typename ConvertedMessageMemoryStrategyT =
      rclcpp::message_memory_strategy::MessageMemoryStrategy<ConvertedMessageT, AllocatorT>,
#if ROS_DISTRO_GALACTIC
    typename SteeSubscriptionT = SteeSubscription<
      MessageT, ConvertedMessageT, AllocatorT, MessageMemoryStrategyT,
      ConvertedMessageMemoryStrategyT>
#else
    typename SteeSubscriptionT = SteeSubscription<
      MessageT, ConvertedMessageT, AllocatorT, typename rclcpp::TypeAdapter<MessageT>::custom_type,
      typename rclcpp::TypeAdapter<MessageT>::ros_message_type, MessageMemoryStrategyT,
      ConvertedMessageMemoryStrategyT>
#endif
    >
  std::shared_ptr<SteeSubscriptionT> create_stee_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos, CallbackT && callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(),
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strategy =
      (MessageMemoryStrategyT::create_default()))
  {
    std::shared_ptr<SubscriptionT> sub{nullptr};
    std::shared_ptr<ConvertedSubscriptionT> converted_sub{nullptr};
    auto stee_sub = std::make_shared<SteeSubscriptionT>();

    if (enable_stee_) {
      using rclcpp::node_interfaces::get_node_topics_interface;
      auto node_topics_interface = get_node_topics_interface(this);
      auto resolved_topic_name = node_topics_interface->resolve_topic_name(topic_name);
      auto converted_topic_name = resolved_topic_name + "/stee";

      auto new_callback = [this, resolved_topic_name,
                           callback](std::unique_ptr<ConvertedMessageT> converted_msg) -> void {
        using ConstRef = const MessageT &;
        using UniquePtr = std::unique_ptr<MessageT, MessageDeleter>;
        using SharedConstPtr = std::shared_ptr<const MessageT>;
        using ConstRefSharedConstPtr = const std::shared_ptr<const MessageT> &;
        using SharedPtr = std::shared_ptr<MessageT>;

        auto not_stop_topic = stop_topics_.find(resolved_topic_name) == stop_topics_.end();

        // We added NOLINT because
        // google/cpplint cannot handle `if constexpr` well.
        // https://github.com/cpplint/cpplint/pull/136 is not applied.
        if constexpr (std::is_same_v<CallbackArgT, ConstRef>) {  // NOLINT
          if (not_stop_topic) {
            set_source_table<MessageT>(resolved_topic_name, &converted_msg);
          }
          callback(converted_msg->body);
        } else if constexpr (std::is_same_v<CallbackArgT, UniquePtr>) {  // NOLINT
          if (not_stop_topic) {
            set_source_table<MessageT>(resolved_topic_name, converted_msg.get());
          }
          auto msg = std::make_unique<MessageT>(std::move(converted_msg->body));
          callback(std::move(msg));
        } else if constexpr (std::is_same_v<CallbackArgT, SharedConstPtr>) {  // NOLINT
          if (not_stop_topic) {
            set_source_table<MessageT>(resolved_topic_name, converted_msg.get());
          }
          auto msg = std::make_shared<const MessageT>(std::move(converted_msg->body));
          callback(msg);
        } else if constexpr (std::is_same_v<CallbackArgT, ConstRefSharedConstPtr>) {  // NOLINT
          if (not_stop_topic) {
            set_source_table<MessageT>(resolved_topic_name, converted_msg.get());
          }
          const auto msg = std::make_shared<const MessageT>(std::move(converted_msg->body));
          callback(msg);
        } else if constexpr (std::is_same_v<CallbackArgT, SharedPtr>) {  // NOLINT
          if (not_stop_topic) {
            set_source_table<MessageT>(resolved_topic_name, converted_msg.get());
          }
          auto msg = std::make_shared<MessageT>(std::move(converted_msg->body));
          callback(msg);
        } else {
          static_assert(always_false_v<CallbackArgT>, "non-exhaustive visitor");
        }
      };

      // TODO(y-okumura-isp): how to prepare converted_msg_mem_strategy
      converted_sub =
        create_subscription<ConvertedMessageT>(converted_topic_name, qos, new_callback, options);

      if (converted_sub->get_publisher_count() == 0) {
        RCLCPP_WARN(this->get_logger(), "no SteePublisher: '%s'", converted_topic_name.c_str());
      }

      stee_sub->set_converted_sub(converted_sub);
    } else {
      sub = create_subscription<MessageT>(topic_name, qos, callback, options, msg_mem_strategy);
      stee_sub->set_sub(sub);
    }

    return stee_sub;
  }

  template <
    typename MessageT, typename ConvertedMessageT = ConvertedMessageType<MessageT>,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>,
    typename ConvertedPublisherT = rclcpp::Publisher<ConvertedMessageT, AllocatorT>,
    typename SteePublisherT = SteePublisher<MessageT, ConvertedMessageT, AllocatorT>>
  std::shared_ptr<SteePublisherT> create_stee_publisher(
    const std::string & topic_name, const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options =
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())
  {
    using rclcpp::node_interfaces::get_node_topics_interface;
    auto node_topics_interface = get_node_topics_interface(this);
    auto resolved_topic_name = node_topics_interface->resolve_topic_name(topic_name);

    auto pub =
      create_publisher<MessageT, AllocatorT, PublisherT>(resolved_topic_name, qos, options);
    auto converted_pub = create_publisher<ConvertedMessageT, AllocatorT, ConvertedPublisherT>(
      resolved_topic_name + "/stee", qos, options);
    auto stee_pub = std::make_shared<SteePublisherT>(
      source_table_, pub, converted_pub, get_fully_qualified_name(), this->get_clock(),
      steady_clock_);
    return stee_pub;
  }

  template <
    typename MessageT, typename ConvertedMessageT = ConvertedMessageType<MessageT>,
    typename AllocatorT = std::allocator<void>,
    typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>,
    typename ConvertedPublisherT = rclcpp::Publisher<ConvertedMessageT, AllocatorT>,
    typename SteePublisherT = SteePublisher<MessageT, ConvertedMessageT, AllocatorT>>
  std::shared_ptr<SteePublisherT> create_stee_republisher(
    const std::string & topic_name, const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options =
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())
  {
    auto converted_pub = create_publisher<ConvertedMessageT, AllocatorT, ConvertedPublisherT>(
      topic_name + "/stee", qos, options);
    auto stee_pub = std::make_shared<SteePublisherT>(
      source_table_, nullptr, converted_pub, get_fully_qualified_name(), this->get_clock(),
      steady_clock_);
    return stee_pub;
  }

private:
  void init();
  std::shared_ptr<rclcpp::Clock> steady_clock_;

  std::shared_ptr<SteeSourcesTable> source_table_;

  /// stop topics for preventing loop topic structure
  /**
   * set this by ROS2 parameter.
   *   key: stee_stop_topics
   *   value: string[]
   *
   * TODO(y-okumura-isp): read this parameter dynamically.
   * We can set it only by startup option for now.
   */
  std::set<std::string> stop_topics_;

  /// Enable STEE or not
  /**
   * We can set this parameter by "enable_stee" only at initialization.
   * This is because if we change "enable/disable" dynamically,
   * we need to dynamically change the message type (original type or STEE type).
   * We cannot do this without re-create the publisher/subscription pair.
   */
  bool enable_stee_{};

  template <class MessageT, class ConvertedMessageT = ConvertedMessageType<MessageT>>
  void set_source_table(const std::string & topic, const ConvertedMessageT * msg)
  {
    auto stamp = Process<MessageT>::get_timestamp_from_const(&(msg->body));

    if (!stamp) {
      return;
    }

    if (msg->sources.size() > 0) {
      source_table_->set(topic, *stamp, msg->sources);
    } else {
      std::vector<tilde_msg::msg::SteeSource> sources;
      tilde_msg::msg::SteeSource source_msg;
      source_msg.topic = topic;
      source_msg.stamp = *stamp;
      source_msg.first_subscription_steady_time = steady_clock_->now();
      sources.emplace_back(std::move(source_msg));
      source_table_->set(topic, *stamp, sources);
    }
  }
};

}  // namespace tilde

#endif  // TILDE__STEE_NODE_HPP_
