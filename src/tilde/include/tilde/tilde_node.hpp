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

#ifndef TILDE__TILDE_NODE_HPP_
#define TILDE__TILDE_NODE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp/message_info.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/get_node_topics_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rmw/types.h"
#include "tilde/tp.h"
#include "tilde_msg/msg/message_tracking_tag.hpp"
#include "tilde_publisher.hpp"

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>

#define TILDE_NODE_GET_PTR(MessageT, msg, out)                            \
  using S = std::decay_t<decltype(msg)>;                                  \
  using ConstRef = const MessageT &;                                      \
  using UniquePtr = std::unique_ptr<MessageT, MessageDeleter>;            \
  using SharedConstPtr = std::shared_ptr<const MessageT>;                 \
  using ConstRefSharedConstPtr = const std::shared_ptr<const MessageT> &; \
  using SharedPtr = std::shared_ptr<MessageT>;                            \
                                                                          \
  if constexpr (std::is_same_v<S, ConstRef>) {                            \
    (out) = &(msg);                                                       \
  } else if constexpr (std::is_same_v<S, UniquePtr>) {                    \
    (out) = (msg).get();                                                  \
  } else if constexpr (std::is_same_v<S, SharedConstPtr>) {               \
    (out) = (msg).get();                                                  \
  } else if constexpr (std::is_same_v<S, ConstRefSharedConstPtr>) {       \
    (out) = (msg).get();                                                  \
  } else if constexpr (std::is_same_v<S, SharedPtr>) {                    \
    (out) = (msg).get();                                                  \
  } else {                                                                \
    static_assert(always_false_v<S>, "non-exhaustive visitor!");          \
  }

namespace tilde
{
template <class>
inline constexpr bool always_false_v = false;

/// Use TildeNode instead of rclcpp::Node and its create_* methods
class TildeNode : public rclcpp::Node
{
  typedef std::map<std::string, std::shared_ptr<TildePublisherBase>> PublisherMap;

public:
  RCLCPP_SMART_PTR_DEFINITIONS(TildeNode)

  /// see corresponding rclcpp::Node constructor
  RCLCPP_PUBLIC
  explicit TildeNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// see corresponding rclcpp::Node constructor
  RCLCPP_PUBLIC
  explicit TildeNode(
    const std::string & node_name, const std::string & namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  RCLCPP_PUBLIC
  virtual ~TildeNode();

  /// create custom subscription
  template <
    typename MessageT, typename CallbackT, typename AllocatorT = std::allocator<void>,
    typename MessageDeleter = std::default_delete<MessageT>,
    typename CallbackMessageT =
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type,
    typename CallbackArgT =
      typename rclcpp::function_traits::function_traits<CallbackT>::template argument_type<0>,
    typename SubscriptionT = rclcpp::Subscription<CallbackMessageT, AllocatorT>,
    typename MessageMemoryStrategyT =
      rclcpp::message_memory_strategy::MessageMemoryStrategy<CallbackMessageT, AllocatorT>>
  std::shared_ptr<SubscriptionT> create_tilde_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos, CallbackT && callback,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>(),
    typename MessageMemoryStrategyT::SharedPtr msg_mem_strategy =
      (MessageMemoryStrategyT::create_default()))
  {
    using rclcpp::node_interfaces::get_node_topics_interface;
    auto node_topics_interface = get_node_topics_interface(this);
    auto resolved_topic_name = node_topics_interface->resolve_topic_name(topic_name);

    auto callback_addr = &callback;

    tracepoint(
      TRACEPOINT_PROVIDER, tilde_subscription_init, callback_addr, get_fully_qualified_name(),
      resolved_topic_name.c_str());

    auto main_topic_callback = [this, resolved_topic_name, callback,
                                callback_addr](CallbackArgT msg) -> void {
      if (this->enable_tilde) {
        auto subscription_time = this->now();
        auto subscription_time_steady = this->steady_clock_->now();

        tracepoint(
          TRACEPOINT_PROVIDER, tilde_subscribe, callback_addr,
          subscription_time_steady.nanoseconds());

        const MessageT * p_msg;
        TILDE_NODE_GET_PTR(MessageT, msg, p_msg);
        register_message_as_input(
          p_msg, resolved_topic_name, subscription_time, subscription_time_steady);
      }
      // finally, call original function
      callback(std::forward<CallbackArgT>(msg));
    };

    return create_subscription<MessageT>(
      topic_name, qos, main_topic_callback, options, msg_mem_strategy);
  }

  template <
    typename MessageT, typename AllocatorT = std::allocator<void>,
    typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>,
    typename TildePublisherT = TildePublisher<MessageT, AllocatorT>>
  std::shared_ptr<TildePublisherT> create_tilde_publisher(
    const std::string & topic_name, const rclcpp::QoS & qos,
    const rclcpp::PublisherOptionsWithAllocator<AllocatorT> & options =
      rclcpp::PublisherOptionsWithAllocator<AllocatorT>())
  {
    auto pub = create_publisher<MessageT, AllocatorT, PublisherT>(topic_name, qos, options);
    auto info_topic = std::string(pub->get_topic_name()) + "/message_tracking_tag";
    auto info_pub =
      create_publisher<TildePublisherBase::InfoMsg>(info_topic, rclcpp::QoS(1), options);

    auto tilde_pub = std::make_shared<TildePublisherT>(
      info_pub, pub, get_fully_qualified_name(), this->get_clock(), steady_clock_,
      this->enable_tilde);
    tilde_pubs_[info_topic] = tilde_pub;

    tracepoint(
      TRACEPOINT_PROVIDER, tilde_publisher_init, tilde_pub.get(), get_fully_qualified_name(),
      pub->get_topic_name());

    return tilde_pub;
  }

  /// register message as input
  /**
   * Register message to the both implicit and explicit data store.
   * Canonically, this is called when subscription gets a message.
   *
   * \param p_msg[in] message raw pointer, can be freed after the function call
   * \param resolved_topic_name[in] topic FQN
   * \param subscription_time subscription time on ROS_TIME
   * \param subscription_time subscription time on steady clock
   */
  template <typename MessageT, typename MessageDeleter = std::default_delete<MessageT>>
  void register_message_as_input(
    const MessageT * p_msg, const std::string & resolved_topic_name,
    const rclcpp::Time & subscription_time, const rclcpp::Time & subscription_time_steady)
  {
    // prepare InputInfo

    auto header_stamp = Process<MessageT>::get_timestamp_from_const(p_msg);

    auto input_info = std::make_shared<InputInfo>();

    input_info->sub_time = subscription_time;
    input_info->sub_time_steady = subscription_time_steady;
    if (header_stamp) {
      input_info->has_header_stamp = true;
      input_info->header_stamp = *header_stamp;
    }

    // TODO(y-okumura-isp): consider race condition in multi threaded executor.
    // i.e. subA comes when subB callback which uses topicA is running
    for (auto & [topic, tp] : tilde_pubs_) {
      tp->set_implicit_input_info(resolved_topic_name, input_info);
      if (input_info->has_header_stamp) {
        tp->set_explicit_subscription_time(resolved_topic_name, input_info);
      }
    }
  }

  /// automatically find subscription_time and subscription_time_steady
  /**
   * Fill subscription_time and subscription_time_steady from internal data.
   * This if for TILDE framework internal use,
   * so users are expected to use explicit API.
   *
   * \param p_msg[in]
   * \param resolved_topic_name[in]
   * \param subscription_time[out]
   * \param subscription_time_steady[out]
   * \return true if subscription_time found else false
   * \sa register_message_as_input
   */
  template <typename MessageT, typename MessageDeleter = std::default_delete<MessageT>>
  bool find_subscription_time(
    const MessageT * p_msg, const std::string & resolved_topic_name,
    rclcpp::Time & subscription_time, rclcpp::Time & subscription_time_steady)
  {
    // get header stamp
    auto header_stamp = Process<MessageT>::get_timestamp_from_const(p_msg);

    InputInfo input_info;

    if (header_stamp) {
      input_info.has_header_stamp = true;
      input_info.header_stamp = *header_stamp;
    }

    bool found = false;
    if (header_stamp && tilde_pubs_.size() > 0) {
      auto pub = tilde_pubs_.begin()->second;
      found = pub->get_input_info(resolved_topic_name, *header_stamp, input_info);
    }

    if (found) {
      subscription_time = input_info.sub_time;
      subscription_time_steady = input_info.sub_time_steady;
    }

    return found;
  }

  rclcpp::Time get_steady_time() { return steady_clock_->now(); }

private:
  /// topic vs publishers
  PublisherMap tilde_pubs_;
  /// node clock may be simulation time
  std::shared_ptr<rclcpp::Clock> steady_clock_;

  /// whether to enable tilde
  // TODO(y-okumura-isp) enable dynamic configuration
  bool enable_tilde;
};

}  // namespace tilde

#undef TILDE_NODE_GET_PTR

#endif  // TILDE__TILDE_NODE_HPP_
